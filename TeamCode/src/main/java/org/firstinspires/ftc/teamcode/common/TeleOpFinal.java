package org.firstinspires.ftc.teamcode.common;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "OTTERRRRR")
public class TeleOpFinal extends LinearOpMode {
    claw claw = new claw();
    ArmSlider ArmSlider = new ArmSlider();
    PID_Arm PID_Arm = new PID_Arm();
    vroomVroom vroom = new vroomVroom();
    PID_Slider PID_Slider = new PID_Slider();
    wrist wrist = new wrist();

    @Override
    public void runOpMode() throws InterruptedException {


        claw.init(hardwareMap);
        ArmSlider.init(hardwareMap);
        PID_Arm.init(hardwareMap);
        vroom.init(hardwareMap);
        PID_Slider.init(hardwareMap);
        wrist.init(hardwareMap);


        waitForStart();

        while (opModeIsActive()) {

            vroom.vrooooooom(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_trigger);
            PID_Arm.math();

            //Rumble and Reset Yaw
            if (gamepad1.options) {
                vroom.resetYaw();
            } else if (gamepad1.dpad_down) {
                gamepad2.rumble(1);
            }
            //claw code
            else if (gamepad1.a) {
                //open
                // 0.4
                claw.set1();
            } else if (gamepad1.b) {
                //close
                //0.7
                claw.set2();
            }
            //Slider Arm

            else if (gamepad2.x) {
                PID_Arm.up();
            } else if (gamepad2.y) {
                PID_Arm.down();
            }


            //Wrist
            else if (gamepad2.b) {
                wrist.set1();
            } else if (gamepad2.a) {
                wrist.set2();
            } else if (gamepad2.dpad_up){
                wrist.wristServo.setPosition(0);
            } else if (gamepad2.options){
                //PID_Arm.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //PID_Arm.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            //PID Sliders
            PID_Slider.sliderMotor.setPower(gamepad2.right_stick_y);
            PID_Slider.sliderMotorMotor.setPower(gamepad2.right_stick_y);
            ArmSlider.armSliderServo.setPower(gamepad2.left_stick_y);

            if (gamepad2.left_stick_y != 0){
            PID_Arm.target = PID_Arm.target + PID_Arm.armticks * 10;
            }
            //heyhey

            /*
            if(gamepad2.right_trigger!= 0) {
                PID_Arm.armMotor.setPower(gamepad2.right_trigger/2.5);
            } else if(gamepad2.left_trigger != 0){
                PID_Arm.armMotor.setPower(-gamepad2.left_trigger/2.5);
            }else{
                PID_Arm.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                PID_Arm.armMotor.setPower(0);
             */
            }


        }


    }







