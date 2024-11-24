//package org.firstinspires.ftc.teamcode.Autonomous.BlueAutonomous;
//
//public class PinPoint /*   MIT License
// *   Copyright (c) [2024] [Base 10 Assets, LLC]
// *
// *   Permission is hereby granted, free of charge, to any person obtaining a copy
// *   of this software and associated documentation files (the "Software"), to deal
// *   in the Software without restriction, including without limitation the rights
// *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// *   copies of the Software, and to permit persons to whom the Software is
// *   furnished to do so, subject to the following conditions:
//
// *   The above copyright notice and this permission notice shall be included in all
// *   copies or substantial portions of the Software.
//
// *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// *   SOFTWARE.
// */
//
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//
//@Autonomous(name = "Auto Left Side")
//public class AutoLeftSide extends LinearOpMode {
//
//    GoBildaPinpointDriver odo;
//    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
//    private DcMotor pivotMotor;
//    private DcMotor sliderMotor;
//    private Servo intakeOutServo, wristServo;
//
//    IMU imu;
//
//    // Slider positions
//    public static final int MAX_POSITION = -2200;
//    public static final int MID_POSITION = -1100;
//    public static final int MIN_POSITION = 50;
//
//    // Pivot positions
//    public static final int VERTICAL_POSITION = -2100;
//    public static final int HORIZONTAL_POSITION = -750;
//    public static final int HOME_POSITION = -100;
//
//    private boolean sliderIsNotFullyRetracted = false;
//
//
//
//    @Override
//    public void runOpMode() {
//
//        // Initialize for Auto
//        initAuto();
//
//        // Wait for the game to start (driver presses START)
//        waitForStart();
//        resetRuntime();
//
//        // Drive to basket
//        driveToPos(300, 550);
//
//        // Turn towards basket
//        gyroTurnToAngle(-45);
//
//        // Rotate Pivot upwards
//        pivotUpwardsAuto();
//
//        // Move Slider to High basket
//        extendToMaxAuto();
//
//        // Rotate wrist servo
//        wristServo.dropPos();
//
//        // Turn intake wheel outward
//        intakeOutServo.dropOff();
//    }
//
//    /*
//    public void rotateTestMotor(int encoderPosition) {
//        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        testMotor.setPower(1);
//        testMotor.setTargetPosition(encoderPosition);
//    }
//    */
//    /*
//    public void rotateWrist(double wristPosition) {
//        testServo.setPosition(wristPosition);
//    }
//    */
//
//
//    public void driveToPos(double targetX, double targetY) {
//        odo.update();
//        boolean telemAdded = false;
//
//        while (opModeIsActive() && ((Math.abs(targetX - odo.getPosX()) > 50)
//                || (Math.abs(targetY - odo.getPosY())) > 50)) {
//            odo.update();
//
//            double x = 0.001*(targetX - odo.getPosX());
//            double y = -0.001*(targetY - odo.getPosY());
//
//            double botHeading = odo.getHeading();
//
//            double rotY = y * Math.cos(-botHeading) - x * Math.sin(-botHeading);
//            double rotX = y * Math.sin(-botHeading) + x * Math.cos(-botHeading);
//
//            if (!telemAdded) {
//                telemetry.addData("x: ", x);
//                telemetry.addData("y: ", y);
//                telemetry.addData("rotX: ", rotX);
//                telemetry.addData("rotY: ", rotY);
//                telemetry.update();
//                telemAdded = true;
//            }
//
//            if (Math.abs(rotX) < 0.15) {
//                rotX = Math.signum(rotX) * 0.15;
//            }
//
//            if (Math.abs(rotY) < 0.15) {
//                rotY = Math.signum(rotY) * 0.15;
//            }
//
//            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
//            double frontLeftPower = (rotX + rotY) / denominator;
//            double backLeftPower = (rotX - rotY) / denominator;
//            double frontRightPower = (rotX - rotY) / denominator;
//            double backRightPower = (rotX + rotY) / denominator;
//
//            frontLeftMotor.setPower(frontLeftPower);
//            backLeftMotor.setPower(backLeftPower);
//            frontRightMotor.setPower(frontRightPower);
//            backRightMotor.setPower(backRightPower);
//
//            telemetry.addData("X: ", odo.getPosX());
//            telemetry.addData("Y: ", odo.getPosY());
//            telemetry.addData("Heading Odo: ", Math.toDegrees(odo.getHeading()));
//            telemetry.addData("Heading IMU: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//            telemetry.update();
//        }
//
//        frontLeftMotor.setPower(0);
//        backLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        backRightMotor.setPower(0);
//    }
//
//    public void gyroTurnToAngle(double turnAngle) {
//        double error, currentHeadingAngle, driveMotorsPower;
//        imu.resetYaw();
//
//        error = turnAngle;
//
//        while (opModeIsActive() && ((error > 1) || (error < -1))) {
//            odo.update();
//            telemetry.addData("X: ", odo.getPosX());
//            telemetry.addData("Y: ", odo.getPosY());
//            telemetry.addData("Heading Odo: ", Math.toDegrees(odo.getHeading()));
//            telemetry.addData("Heading IMU: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//            telemetry.update();
//
//            driveMotorsPower = error / 200;
//
//            if ((driveMotorsPower < 0.2) && (driveMotorsPower > 0)) {
//                driveMotorsPower = 0.2;
//            } else if ((driveMotorsPower > -0.2) && (driveMotorsPower < 0)) {
//                driveMotorsPower = -0.2;
//            }
//
//            // Positive power causes left turn
//            frontLeftMotor.setPower(-driveMotorsPower);
//            backLeftMotor.setPower(-driveMotorsPower);
//            frontRightMotor.setPower(driveMotorsPower);
//            backRightMotor.setPower(driveMotorsPower);
//
//            currentHeadingAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//            error = turnAngle - currentHeadingAngle;
//        }
//        frontLeftMotor.setPower(0);
//        backLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        backRightMotor.setPower(0);
//    }
//
//    public void initAuto() {
//        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
//        odo.setOffsets(132, 88); //these are tuned for 3110-0002-0001 Product Insight #1
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
//                GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        odo.resetPosAndIMU();
//
//        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
//        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
//        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
//        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
//        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        pivotMotor = hardwareMap.dcMotor.get("pivotMotor");
//        sliderMotor = hardwareMap.dcMotor.get("sliderMotor");
//        intakeOutServo = hardwareMap.dcMotor.get("sliderMotor");
//
//        // Retrieve the IMU from the hardware map
//        imu = hardwareMap.get(IMU.class, "imu");
//        // Adjust the orientation parameters to match your robot
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
//        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//        imu.initialize(parameters);
//        imu.resetYaw();
//    }
//
//    public void extendToMaxAuto() {
//        double sliderPower = 0;
//        sliderMotor.setPower(-0.7);
//        sliderMotor.setTargetPosition(MAX_POSITION);
//        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        sliderIsNotFullyRetracted = true;
//        while (sliderMotor.getCurrentPosition() > MAX_POSITION) {
//
//        }
//    }
//
//    public void retractToMinAuto() {
//        double sliderPower = 0;
//        sliderMotor.setPower(0.4);
//        sliderMotor.setTargetPosition(MIN_POSITION);
//        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        sliderIsNotFullyRetracted = false;
//        while (sliderMotor.getCurrentPosition() < MIN_POSITION) {
//
//        }
//    }
//
//
//    public void pivotUpwardsAuto() {
//        if (!isPivotUp) {
//            pivotMotor.setPower(0.4);
//            pivotMotor.setTargetPosition(VERTICAL_POSITION);
//            pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//        isPivotUp = true;
//
//        while (pivotMotor.getCurrentPosition() > VERTICAL_POSITION) {
//
//        }
//    }
//
//    public void pivotDownwardsAuto() {
//        if (isPivotUp) {
//            pivotMotor.setPower(0.3);
//            pivotMotor.setTargetPosition(HOME_POSITION);
//            pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//        isPivotUp = false;
//
//        while (pivotMotor.getCurrentPosition() < HOME_POSITION) {
//
//        }
//    }
//}
//
//{
//}
