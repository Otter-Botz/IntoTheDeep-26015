package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.teamcode.common.RobotController.SLIDER_HOLD_SPEED;
import static org.firstinspires.ftc.teamcode.common.RobotController.SLIDER_UP_SPEED;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.RobotController;
@TeleOp(name = "TeleOpDriver", group = "OtterBotz")
public class TeleOpDriver extends LinearOpMode {
    boolean isSliderHold = false;
    private RobotController RobotController;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotController = new RobotController(this);
        RobotController.initTeleOp();

        telemetry.addData("Status", "Ready to run");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // slider controls
            controlSlider();

            driveBase();


            // Reset
            if (gamepad1.guide) {
                RobotController.resetIMU();

            }


            telemetry.addData("Status", "Running....");
            telemetry.update();
        }


    }

    private void driveBase(){
        double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;
        if(rx == 0 && gamepad1.dpad_left){
            rx = 0.25;
        }
        if(rx == 0 && gamepad1.dpad_right){
            rx = -0.25;
        }

        if(y == 0 && gamepad1.dpad_up){
            y = -0.25;
        }
        if(y == 0 && gamepad1.dpad_down){
            y = 0.25;
        }

    }

    public void sliderDrivePosition() {
        org.firstinspires.ftc.teamcode.common.RobotController.linearSlide.setTargetPosition(100);
        org.firstinspires.ftc.teamcode.common.RobotController.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        org.firstinspires.ftc.teamcode.common.RobotController.linearSlide.setPower(SLIDER_UP_SPEED);
        while (opModeIsActive() && org.firstinspires.ftc.teamcode.common.RobotController.linearSlide.isBusy()) {
        }
        org.firstinspires.ftc.teamcode.common.RobotController.linearSlide.setPower(SLIDER_HOLD_SPEED);
        org.firstinspires.ftc.teamcode.common.RobotController.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void controlSlider() {

        boolean isSliderHold = false;
        if (gamepad2.left_bumper) { // line 1 position
            org.firstinspires.ftc.teamcode.common.RobotController.linearSlide.setTargetPosition(1400);
            org.firstinspires.ftc.teamcode.common.RobotController.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            org.firstinspires.ftc.teamcode.common.RobotController.linearSlide.setPower(SLIDER_UP_SPEED);
            while (opModeIsActive() && org.firstinspires.ftc.teamcode.common.RobotController.linearSlide.isBusy()) {

            }
            org.firstinspires.ftc.teamcode.common.RobotController.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else if (gamepad2.right_bumper) { // line 2 position
            org.firstinspires.ftc.teamcode.common.RobotController.linearSlide.setTargetPosition(1750);
            org.firstinspires.ftc.teamcode.common.RobotController.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            org.firstinspires.ftc.teamcode.common.RobotController.linearSlide.setPower(SLIDER_UP_SPEED);
            while (opModeIsActive() && org.firstinspires.ftc.teamcode.common.RobotController.linearSlide.isBusy()) {

            }
            isSliderHold = false;
            org.firstinspires.ftc.teamcode.common.RobotController.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else if (gamepad2.left_trigger != 0) { // up
            isSliderHold = false;
            org.firstinspires.ftc.teamcode.common.RobotController.moveSliderUp();
        }
        else if (gamepad2.right_trigger != 0) { // down
            isSliderHold = false;
            org.firstinspires.ftc.teamcode.common.RobotController.moveSliderDown();
        }
        else if (!isSliderHold){
            makeSliderIdle();
        }
    }

    private void makeSliderIdle() {

        org.firstinspires.ftc.teamcode.common.RobotController.setSliderIdlePosition();
    }


}

