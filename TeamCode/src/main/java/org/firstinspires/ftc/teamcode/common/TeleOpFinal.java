package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

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
            //o
        while (opModeIsActive()) {
            vroom.vrooooooom(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, gamepad1.right_trigger);
            PID_Arm.math();

            if (gamepad1.options) {
                vroom.resetYaw();
            } else if (gamepad1.dpad_down) {
                gamepad2.rumble(1);
            } else if (gamepad1.a) {
                //open
                claw.set1();
            } else if (gamepad1.b) {
                //close
                claw.set2();
            }
        /*
        else if (gamepad2.x && PID_Arm.target == 300){
            PID_Arm.up();
        }
        else if (gamepad2.x && PID_Arm.target == 600){
            PID_Arm.down();
        }
        */

        /*
        else if (gamepad2.b) {
            wrist.set1();
        } else if (gamepad2.b) {
            wrist.set2();
        }
        */
         ArmSlider.armSliderServo.setPower(gamepad2.left_stick_y);
            //what

        }







    }
}






