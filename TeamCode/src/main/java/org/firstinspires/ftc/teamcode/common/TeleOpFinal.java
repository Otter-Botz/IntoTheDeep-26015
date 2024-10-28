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
        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");


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
            } else if (gamepad2.dpad_down) {
                PID_Arm.armMotor.setPower(-0.2);
            } else if (gamepad2.dpad_up) {
                PID_Arm.armMotor.setPower(0.2);
            }
            //Wrist
            else if (gamepad2.b) {
                wrist.set1();
            } else if (gamepad2.a) {
                wrist.set2();
            }
            //PID Sliders
            PID_Slider.sliderMotor.setPower(-gamepad2.right_stick_y);
            PID_Slider.sliderMotorMotor.setPower(-gamepad2.right_stick_y);
            ArmSlider.armSliderServo.setPower(gamepad2.left_stick_y);


        }


    }
}






