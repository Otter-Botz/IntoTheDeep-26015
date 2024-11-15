package org.firstinspires.ftc.teamcode.common;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.teamcode.Autonomous.BlueAutonomous.Biddle4Specimen;

@TeleOp(name = "OTTERRRRR")
public class TeleOpFinal extends LinearOpMode {
    claw claw = new claw();
    ArmSlider ArmSlider = new ArmSlider();
    PID_Arm PID_Arm = new PID_Arm();
    vroomVroom vroom = new vroomVroom();
    Slider Slider = new Slider();
    wrist wrist = new wrist();
    TouchSensor touchSensor;


    @Override
    public void runOpMode() throws InterruptedException {

        touchSensor = hardwareMap.get(TouchSensor.class, "sensorTouch");
        claw.init(hardwareMap);
        ArmSlider.init(hardwareMap);
        PID_Arm.init(hardwareMap);
        vroom.init(hardwareMap);
        Slider.init(hardwareMap);
        wrist.init(hardwareMap);
        boolean clawPressed = false;
        boolean wristPressed = false;
        boolean isPressed = touchSensor.isPressed();

        waitForStart();

        while (opModeIsActive()) {

            vroom.vrooooooom(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_trigger);
            PID_Arm.math();

            if (PID_Arm.target < 500) {
               PID_Arm.armRespond(gamepad2.left_stick_y);
          }

            if (touchSensor.isPressed() && gamepad2.dpad_right) {
              PID_Arm.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
              PID_Arm.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }


            //Rumble and Reset Yaw
            if (gamepad1.options) {
                vroom.resetYaw();
            }

            if (gamepad1.dpad_down) {
                gamepad2.rumble(1000);
            }
            wristPressed = wristPressed && gamepad2.a;
            clawPressed = clawPressed && gamepad1.b;
            if(gamepad2.a && !wristPressed){
                toggleWrist();
                wristPressed = true;
            }
            if(gamepad1.b && !clawPressed){
                toggleClaw();
                clawPressed = true;
            }
            //claw code

           //Slider Arm
            if (gamepad2.x) {
                PID_Arm.up();
            } else if (gamepad2.y) {
                PID_Arm.down();
            }



            else if (gamepad2.dpad_up){
                highbasket();
            }
            else if (gamepad2.dpad_down){
                submersible();
            }


            Slider.sliderMotor.setPower(-gamepad2.right_stick_y);
            Slider.sliderMotorMotor.setPower(-gamepad2.right_stick_y);
            ArmSlider.armSliderServo.setPower(gamepad2.left_stick_y);


            if (gamepad2.left_bumper) {
                claw.clawServo.setPosition(0);
            } else if (gamepad2.right_bumper) {
                wrist.wristServo.setPosition(0);
            }



            // back up manual arm control
            /*
            if(gamepad2.right_trigger!= 0) {
                PID_Arm.armMotor.setPower(gamepad2.right_trigger/2.5);
            } else if(gamepad2.left_trigger != 0){
                PID_Arm.armMotor.setPower(-gamepad2.left_trigger/2.5);
            }else {
                PID_Arm.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                PID_Arm.armMotor.setPower(0);
            }

             */
            //telemetry.addData("pos", PID_Arm.armMotor.getCurrentPosition());
            //telemetry.update();


        }


    }
    private  void toggleClaw(){
        if(claw.getPosition() == claw.close){
            claw.set(claw.open);
        } else if(claw.getPosition() != claw.close){
            claw.set(claw.close);
        }
    }
    private  void toggleWrist(){
        if(wrist.getPosition() == wrist.up){
            wrist.set(wrist.down);
        } else if(wrist.getPosition() != wrist.up){
            wrist.set(wrist.up);
        }
    }
    public void highbasket() {
        Slider.sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slider.sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slider.sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider.sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider.sliderMotor.setTargetPosition(2000);
        Slider.sliderMotorMotor.setTargetPosition(2000);
        Slider.sliderMotor.setPower(0.5);
        Slider.sliderMotorMotor.setPower(0.5);
        PID_Arm.up();
        wrist.wristServo.setPosition(0.4);
    }

    public void highrung() {
        Slider.sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slider.sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slider.sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider.sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider.sliderMotor.setTargetPosition(300);
        Slider.sliderMotorMotor.setTargetPosition(300);
        PID_Arm.armMotor.setTargetPosition(812);
        wrist.wristServo.setPosition(0.1);

    }

    public void submersible() {
        Slider.sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slider.sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slider.sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider.sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider.sliderMotor.setTargetPosition(100);
        Slider.sliderMotorMotor.setTargetPosition(100);
        PID_Arm.down();
        claw.set(claw.open);
        wrist.set(wrist.up);
    }

}








