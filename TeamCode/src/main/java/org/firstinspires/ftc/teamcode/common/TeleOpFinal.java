package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "OTTERRRRR")
public class TeleOpFinal extends LinearOpMode {
    claw claw = new claw();
    ArmSlider ArmSlider = new ArmSlider();
    PID_Arm PID_Arm = new PID_Arm();
    vroomVroom vroom = new vroomVroom();
    Slider Slider = new Slider();
    wrist wrist = new wrist();
    private LinearOpMode linearOpMode;
    double subArmPos = 300;

    @Override
    public void runOpMode() throws InterruptedException {

        claw.init(hardwareMap);
        ArmSlider.init(hardwareMap);
        PID_Arm.init(hardwareMap);
        vroom.init(hardwareMap);
        Slider.init(hardwareMap);
        wrist.init(hardwareMap);

        waitForStart();

        PID_Arm.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PID_Arm.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            telemetry.addData("pos", PID_Arm.armMotor.getCurrentPosition());
            telemetry.addData("servopos", wrist.wristServo.getPosition());
            telemetry.update();

            //Drive
            vroom.vrooooooom(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_trigger);

            //Math
            PID_Arm.math();



            //Gamepad 1
            if (gamepad1.options) {
                vroom.resetYaw();
            }
            if (gamepad1.right_stick_button) {
                specimen();
            }
            if (gamepad1.a) {
                claw.set(claw.open);
            }
            if(gamepad1.b){
                claw.set(claw.close);
            }
            if (gamepad1.left_bumper) {
                ArmSlider.set(ArmSlider.out);
                PID_Arm.target = subArmPos;
            }
            if (gamepad1.right_bumper) {
                ArmSlider.set(ArmSlider.in);
                PID_Arm.target = subArmPos;
            }
            if (gamepad1.dpad_right) {
                PID_Arm.target = subArmPos;
                ArmSlider.set(ArmSlider.middle);
            }
            if (gamepad1.left_stick_button) {
                PID_Arm.target = 200;
                wrist.set(wrist.down);
            }
            if (gamepad1.left_trigger > 0){
                double value2;
                value2 = PID_Arm.target - 4*gamepad1.left_trigger;
                PID_Arm.target = value2;
            }


            //Gamepad 2
            if (gamepad2.right_bumper) {
                claw.set(claw.open);
            }
            if (gamepad2.b) {
                highbasket();
            }
            if (gamepad2.a) {
                submersible();
            }
            if (gamepad2.y) {
                ArmUpSliderOut();
            }
            if (gamepad2.dpad_up) {
                ArmSlider.set(ArmSlider.in);
            }
            if (gamepad2.options) {
                PID_Arm.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                PID_Arm.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (gamepad2.right_trigger == 1) {
                double value;
                value = PID_Arm.target + 4;
                PID_Arm.target = value;
            }
            if (gamepad2.left_trigger == 1){
                double value1;
                value1 = PID_Arm.target - 4;
                PID_Arm.target = value1;
            }

            Slider.sliderMotor.setPower(gamepad2.right_stick_y);
            Slider.sliderMotorMotor.setPower(gamepad2.right_stick_y);

            //Zero Code
//            if (gamepad2.left_bumper) {
//                claw.clawServo.setPosition(0.1);
//            } else if (gamepad2.left_bumper) {
//                wrist.wristServo.setPosition(0);
//            }

        }
    }

    public void ArmUpSliderOut() {
        PID_Arm.up();
        ArmSlider.set(ArmSlider.out);
        wrist.set(wrist.up);
    }

    public void highbasket() {
        PID_Arm.up();
        wrist.set(wrist.up);
    }

    public void specimen() {
        PID_Arm.specimen();
        wrist.set(wrist.up);
    }

    public void submersible() {
        PID_Arm.down();
        wrist.set(wrist.down);
    }
}













