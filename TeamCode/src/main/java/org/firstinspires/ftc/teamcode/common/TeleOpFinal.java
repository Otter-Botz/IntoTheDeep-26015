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

@TeleOp(name = "OTTERRRRR")
public class TeleOpFinal extends LinearOpMode {
    claw claw = new claw();
    ArmSlider ArmSlider = new ArmSlider();
    PID_Arm PID_Arm = new PID_Arm();
    vroomVroom vroom = new vroomVroom();
    Slider Slider = new Slider();
    wrist wrist = new wrist();
    TouchSensor touchSensor;

    public void highbasket() {
        Slider.sliderMotor.setTargetPosition(900);
        Slider.sliderMotorMotor.setTargetPosition(-900);
        //PID_Arm.armMotor.setTargetPosition(812);
        wrist.wristServo.setPosition(0.1);
    }

    public void highrung() {
        Slider.sliderMotor.setTargetPosition(700);
        Slider.sliderMotorMotor.setTargetPosition(-700);
        //PID_Arm.armMotor.setTargetPosition(812);
        wrist.wristServo.setPosition(0.1);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        touchSensor = hardwareMap.get(TouchSensor.class, "sensorTouch");


        claw.init(hardwareMap);
        ArmSlider.init(hardwareMap);
        PID_Arm.init(hardwareMap);
        vroom.init(hardwareMap);
        Slider.init(hardwareMap);
        wrist.init(hardwareMap);


        waitForStart();

        while (opModeIsActive()) {

            vroom.vrooooooom(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_trigger);
           //
            //PID_Arm.math();

//            if (PID_Arm.target < 100) {
//                PID_Arm.armRespond(gamepad2.left_stick_y);
//            }

//            if (touchSensor.isPressed()) {
//              PID_Arm.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//              PID_Arm.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
            //Rumble and Reset Yaw
            if (gamepad1.options) {
                vroom.resetYaw();
            }

            if (gamepad1.dpad_down) {
                gamepad2.rumble(1000);
            }
            //claw code
           if (gamepad1.a) {
                //open
                // 0.4
                claw.set1();
            }
           else if (gamepad1.b) {
                //close
                //0.7
                claw.set2();
            }
           //Slider Arm
//            if (gamepad2.x) {
//                PID_Arm.up();
//            } else if (gamepad2.y) {
//            PID_Arm.down();
//            } else if (gamepad2.options) {
//
//            }

            //Wrist
            if (gamepad2.b) {
                wrist.set1();
            } else if (gamepad2.a) {
                wrist.set2();
            }

//            if (gamepad2.options) {
//                PID_Arm.target = -104;
//            }

            else if (gamepad2.dpad_up){
                highbasket();
            }
            else if (gamepad2.dpad_down){
                highrung();
            }

            //PID Sliders
            Slider.sliderMotor.setPower(-gamepad2.right_stick_y);
            Slider.sliderMotorMotor.setPower(-gamepad2.right_stick_y);
           // ArmSlider.armSliderServo.setPower(gamepad2.left_stick_y);

           /* if (gamepad2.left_stick_y != 0){
            PID_Arm.target = PID_Arm.target + PID_Arm.armticks * 10;
            }*/

            if (gamepad2.left_bumper) {
                claw.clawServo.setPosition(0);
            } else if (gamepad2.right_bumper) {
                wrist.wristServo.setPosition(0);
            }

            /*
            if(gamepad2.right_trigger!= 0) {
                PID_Arm.armMotor.setPower(gamepad2.right_trigger/2.5);
            } else if(gamepad2.left_trigger != 0){
                PID_Arm.armMotor.setPower(-gamepad2.left_trigger/2.5);
            }else{
                PID_Arm.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                PID_Arm.armMotor.setPower(0);
             */
            telemetry.addData("pos", PID_Arm.armMotor.getCurrentPosition());
            telemetry.update();


        }


    }
}








