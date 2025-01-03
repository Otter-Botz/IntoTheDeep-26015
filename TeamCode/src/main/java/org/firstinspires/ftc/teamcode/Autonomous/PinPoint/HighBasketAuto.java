package org.firstinspires.ftc.teamcode.Autonomous.PinPoint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "HighBasket")
public class HighBasketAuto extends LinearOpMode {

    int ticksPerInchForward = 23;
    int ticksPerInchSideways = 22;

    //Sliders
    private int minRange = 5;
    private int maxRange = -10;

    // logic: Set target positions for motors
    int targetPosition = (minRange + maxRange) / 2; // Midpoint of range

    @Override
    public void runOpMode() throws InterruptedException {

        AutoCommonClass otterBotzCommon = new AutoCommonClass(this);
        otterBotzCommon.initAuto();

        while (opModeInInit()) {
            telemetry.addData("pos motor1", otterBotzCommon.sliderMotor.getCurrentPosition());
            telemetry.addData("pos motor2", otterBotzCommon.sliderMotorMotor.getCurrentPosition());
            telemetry.addData("pos motor1 target", otterBotzCommon.sliderMotor.getTargetPosition());
            telemetry.addData("pos motor2 target", otterBotzCommon.sliderMotorMotor.getTargetPosition());

            telemetry.addData("PosX()", otterBotzCommon.odo.getPosX());
            telemetry.addData("PosY()", otterBotzCommon.odo.getPosY());
            telemetry.addData("yaw", otterBotzCommon.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

            // Display telemetry for driver adjustment
            telemetry.addLine("Adjust using gamepad:");
            telemetry.addData("Min Range", minRange + " ticks (D-Pad Up/Down)");
            telemetry.addData("Max Range", maxRange + " ticks (D-Pad Left/Right)");
            telemetry.update();


            // Adjust minRange using D-Pad
            if (gamepad1.dpad_down) {
                minRange -= 2; // Decrease minRange
            } else if (gamepad1.dpad_up) {
                minRange += 2; // Increase minRange
            }

            // Adjust maxRange using D-Pad
            if (gamepad1.dpad_left) {
                maxRange -= 2; // Decrease maxRange
            } else if (gamepad1.dpad_right) {
                maxRange += 2; // Increase maxRange
            }

            // Clamp ranges to valid encoder values
            minRange = Math.max(minRange, 0);
            maxRange = Math.max(maxRange, minRange);
        }

        // Wait
        waitForStart();
        resetRuntime();

        //X = Y and Y = x
        //Make sure claw is able to hold sample
        otterBotzCommon.sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        otterBotzCommon.sliderMotorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        otterBotzCommon.sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        otterBotzCommon.sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        otterBotzCommon.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        otterBotzCommon.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        otterBotzCommon.sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        otterBotzCommon.sliderMotorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        otterBotzCommon.odo.resetPosAndIMU();
        sleep(300);
        otterBotzCommon.set(otterBotzCommon.ClawClose);
        otterBotzCommon.set(otterBotzCommon.WristUp);
        sleep(100);
        otterBotzCommon.set(otterBotzCommon.ArmSliderIn);

        // drive to basket
//        otterBotzCommon.driveToPos(-ticksPerInchForward * 10, ticksPerInchSideways * 10);
//        sleep(300);
//        otterBotzCommon.driveToPos(-ticksPerInchForward * 20, ticksPerInchSideways * 10);
//        otterBotzCommon.gyroTurnToAngle(40);
//        // 24.5 Barely Making It In Basket
//        otterBotzCommon.driveToPos(-ticksPerInchForward * 24.5,ticksPerInchSideways * 8);

        //Score 1
//        otterBotzCommon.scoreHighBasket();
        int minPos = 0;    // Minimum encoder count
        int maxPos = 2000; // Maximum encoder count

        otterBotzCommon.sliderUpElapsedTime(500);
//        otterBotzCommon.moveSliders(-500,0.2);
//        //otterBotzCommon.moveSlidersToPositionInRange(otterBotzCommon.sliderMotor, otterBotzCommon.sliderMotorMotor, 500, 500, minPos, maxPos, 0.8, this);sleep(1000);
//        otterBotzCommon.slidersDown();

//        otterBotzCommon.moveToPosition(10,10);
        sleep(500);
        otterBotzCommon.sliderDownElapsedTime();
//        otterBotzCommon.armDownSliderOut();
//        sleep(1000);
//        otterBotzCommon.armUpSliderIn();
//        sleep(1000);

        //Old Code
//        sleep(500);
//        //Slider Down
//        armDown();
//        headingCorrectBasket();
//        wrist.set(wrist.up);

        //Move to first sample
//        otterBotzCommon.armDown();
//        otterBotzCommon.driveToPos(-ticksPerInchForward * 19.5, ticksPerInchSideways * 26.5);
//        otterBotzCommon.set(otterBotzCommon.ClawOpen);
//        sleep(100);
//        otterBotzCommon.driveToPos(-ticksPerInchForward * 24, ticksPerInchSideways * 37);

        /*
        //Pick Up first Sample
//        wrist.set(wrist.AutoUp);
//        sleep(200);
        otterBotzCommon.armDown();
        otterBotzCommon.set(otterBotzCommon.ArmSliderOut);
        sleep(200);
        otterBotzCommon.set(otterBotzCommon.WristMiddle);
        sleep(200);
        otterBotzCommon.set(otterBotzCommon.ClawClose);
        sleep(300);
        otterBotzCommon.set(otterBotzCommon.WristSubmersible);
        sleep(200);

        // move to basket
        otterBotzCommon.driveToPos(-ticksPerInchForward * 28, ticksPerInchSideways * 26.5);
        sleep(200);
        otterBotzCommon.driveToPos(-ticksPerInchForward * 31,ticksPerInchSideways * 14);
        otterBotzCommon.gyroTurnToAngle(-39);


        //Score 2
        otterBotzCommon.sliderUp();
        sleep(100);
        //Slider Down
        otterBotzCommon.armDown();
        //wrist.set(wrist.down);


        //Move to second sample
        //26.5 Sometimes Work
        otterBotzCommon.gyroTurnToAngle(45);
        otterBotzCommon.driveToPos(-ticksPerInchForward * 32, ticksPerInchSideways * 26.5);
        sleep(500);
        otterBotzCommon.set(otterBotzCommon.ClawOpen);
        otterBotzCommon.driveToPos(-ticksPerInchForward * 34.5, ticksPerInchSideways * 39.5);


        //Pick Up Second Sample
//        wrist.set(wrist.AutoUp);
//        sleep(200);
        otterBotzCommon.armDown();
        sleep(200);
        otterBotzCommon.set(otterBotzCommon.WristMiddle);
        sleep(200);
        otterBotzCommon.set(otterBotzCommon.ClawClose);
        sleep(300);
        otterBotzCommon.set(otterBotzCommon.WristSubmersible);
        sleep(200);


        //Move to basket
        otterBotzCommon.driveToPos(-ticksPerInchForward * 31, ticksPerInchSideways * 26.5);
        sleep(500);
        otterBotzCommon.driveToPos(-ticksPerInchForward * 31,ticksPerInchSideways * 14);
        otterBotzCommon.gyroTurnToAngle(-39);

        //Score 3
        otterBotzCommon.lowbasketsliderwithoutwrist();
        sleep(100);
        //Slider Down
        otterBotzCommon.armDown();


        //Park
//        armDown();
//        driveToPos(-ticksPerInchForward * 7, ticksPerInchSideways * 40);
//        sleep(300);
//        driveToPos(-ticksPerInchForward * 3, ticksPerInchSideways * 50);
//        sleep(300);
//        driveToPos(-ticksPerInchForward * 3, ticksPerInchSideways * 60);
//        sleep(1000);

         */

    }
}


