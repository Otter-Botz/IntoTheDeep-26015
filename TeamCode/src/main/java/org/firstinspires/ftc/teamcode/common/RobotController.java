package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class RobotController {
    private LinearOpMode linearOpMode;
    private HardwareMap hardwareMap;
    private VisionProcessor visionProcessor;
    public static final String BLUE_ALLIANCE = "BLUE_ALLIANCE";
    public static final String RED_ALLIANCE = "RED_ALLIANCE";

    // Drive motors
    private DcMotor frontLeftDriveMotor;
    private DcMotor rearLeftDriveMotor;
    private DcMotor frontRightDriveMotor;
    private DcMotor rearRightDriveMotor;
    private DcMotor intakeMotor;

    static final double DELIVERY_PLATE_DRIVE_POS = 0.3;
    private static final double INTAKE_SPEED = 0.6;

    //Servo
    private CRServo rightScoringServo;
    private CRServo leftScoringServo;
    private Servo droneLauncher;
    private Servo rightPlate;
    private Servo leftPlate;
    private static final double DRONE_LAUNCHER_SPEED = 0.4;
    private static final double SERVO_SPEED = 1;

    // Hanging Motor
    private DcMotor hangMotor;
    private static final double HANGING_MOTOR_UP_SPEED = 1.0;
    private static final double HANGING_MOTOR_DOWN_SPEED = 1.0;
    private static final double HANGING_MOTOR_IDLE_SPEED = 0;

    // Slider
    public DcMotor linearSlide;
    public static final double SLIDER_UP_SPEED = 1.0;
    public static final double SLIDER_DOWN_SPEED = 0.5;
    public static final double SLIDER_HOLD_SPEED = 0.001;

    // IMU
    public IMU expImu;
    //    public BNO055IMU expImu;
    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    // AprilTag
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    public VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private final double DESIRED_DISTANCE = 7.0; //  this is how close the camera should get to the target (inches)
    final double SPEED_GAIN  =  0.02;  // 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015; // 0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01; // 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    private static final double IDLE_SPEED = 0.0;

    private static final double COUNTS_PER_MOTOR_REV = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    private static final double DRIVE_GEAR_REDUCTION = 1.0 ;     // No External Gearing.
    private static final double WHEEL_DIAMETER_INCHES = 4.0 ;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    private static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    private static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable
    private double          headingError  = 0;
    private double  targetHeading = 0;
    private double  turnSpeed     = 0;
    private static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    private static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
    private static final double     DELIVERY_PLATE_STOW_POSITION = 0.75;
    private static final double DELIVERY_PLATE_INTAKE_POS = 0.37;
    private static final double DELIVERY_PLATE_SCORING_POS = 1;

    public RobotController(LinearOpMode callingLinearOpMode) {
        this.linearOpMode = callingLinearOpMode;
        this.hardwareMap = callingLinearOpMode.hardwareMap;
    }

    public void initTeleOp(){
        initDriveMotors();
        initIntakeMotor();
        initHangingMotor();
        initServos();
        initLinearSlide();
        //initIMU();
        initExpIMU();
        initVisionProcessor();

    }

    public void initBlueAllianceAuto(){
        initDriveMotors();
        initIntakeMotor();
        initServos();
        initLinearSlide();
        initExpIMU();
        initVisionProcessor(BLUE_ALLIANCE);
    }

    public void initRedAllianceAuto(){
        initDriveMotors();
        initIntakeMotor();
        initServos();
        initLinearSlide();
        initExpIMU();
        initVisionProcessor(RED_ALLIANCE);
    }

    public void end(){
        resetExpIMU();
        visionProcessor.end();
    }

    public void moveTeleOp(double x, double y, double rx, double yaw){
        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-yaw) - y * Math.sin(-yaw);
        double rotY = x * Math.sin(-yaw) + y * Math.cos(-yaw);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftDriveMotor.setPower(frontLeftPower);
        rearLeftDriveMotor.setPower(backLeftPower);
        frontRightDriveMotor.setPower(frontRightPower);
        rearRightDriveMotor.setPower(backRightPower);
    }

    public void moveToAprilTag(int desiredTagId, boolean isAuto){
        desiredTag =    visionProcessor.getAprilTagDetection(desiredTagId);

        // If X is being pressed, AND we have found the desired target, Drive to target Automatically .
        if ((isAuto || linearOpMode.gamepad1.x) && desiredTag != null) {
            sliderDrivePositionAuto();

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double  headingError    = desiredTag.ftcPose.bearing;
            double  yawError        = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            double drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            int moveCounts = (int)(rangeError * COUNTS_PER_INCH);

            // Calculate target encoder positions for each wheel
            int targetFrontLeft = frontLeftDriveMotor.getCurrentPosition() + moveCounts;
            int targetFrontRight = frontRightDriveMotor.getCurrentPosition() + moveCounts;
            int targetRearLeft = rearLeftDriveMotor.getCurrentPosition() + moveCounts;
            int targetRearRight = rearRightDriveMotor.getCurrentPosition() + moveCounts;

            moveToTargetPosition(targetFrontLeft, targetFrontRight, targetRearLeft, targetRearRight);

            while (isOpModeActive() && (frontLeftDriveMotor.isBusy()
                    || frontRightDriveMotor.isBusy()
                    || rearLeftDriveMotor.isBusy()
                    || rearRightDriveMotor.isBusy())) {
                // Continue moving
                moveRobot(drive, strafe, turn);
            }
            stopDriveMotors();
        }
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        frontLeftDriveMotor.setPower(leftFrontPower);
        frontRightDriveMotor.setPower(rightFrontPower);
        rearLeftDriveMotor.setPower(leftBackPower);
        rearRightDriveMotor.setPower(rightBackPower);
    }

    public void moveRobot(double drive, double turn) {
        double driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        double turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        double leftSpeed  = drive - turn;
        double rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        frontLeftDriveMotor.setPower(leftSpeed);
        rearLeftDriveMotor.setPower(leftSpeed);
        frontRightDriveMotor.setPower(rightSpeed);
        rearRightDriveMotor.setPower(rightSpeed);
    }

    public void driveStraight(double maxDriveSpeed,
                              double distance, double desiredHeading) {

        // Ensure that the OpMode is still active
        if (isOpModeActive()) {
            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            int frontLeftTarget = frontLeftDriveMotor.getCurrentPosition() + moveCounts;
            int frontRightTarget = frontRightDriveMotor.getCurrentPosition() + moveCounts;
            int rearLeftTarget = rearLeftDriveMotor.getCurrentPosition() + moveCounts;
            int rearRightTarget = rearRightDriveMotor.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            moveToTargetPosition(frontLeftTarget,frontRightTarget, rearLeftTarget, rearRightTarget);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (isOpModeActive() &&
                    (frontLeftDriveMotor.isBusy() && frontRightDriveMotor.isBusy() && rearRightDriveMotor.isBusy() && rearLeftDriveMotor.isBusy())) {

                double turnSpeed = getSteeringCorrection(desiredHeading, P_DRIVE_GAIN);
                linearOpMode.telemetry.addData("Gyro Heading: ", getHeadingInDegrees());
                linearOpMode.telemetry.addData("Motor Pos: ", (frontLeftDriveMotor.getCurrentPosition() + frontRightDriveMotor.getCurrentPosition() + rearLeftDriveMotor.getCurrentPosition() + rearRightDriveMotor.getCurrentPosition())/4);
                linearOpMode.telemetry.update();

                // Apply the turning correction to the current driving speed.
                moveRobot(maxDriveSpeed, turnSpeed);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            stopDriveMotors();
        }
    }

    public void moveLeft(double power, double distance) {
        int moveCounts = (int)(distance * COUNTS_PER_INCH);

        // Calculate target encoder positions for each wheel
        int targetFrontLeft = frontLeftDriveMotor.getCurrentPosition() - moveCounts;
        int targetFrontRight = frontRightDriveMotor.getCurrentPosition() + moveCounts;
        int targetBackLeft = rearLeftDriveMotor.getCurrentPosition() + moveCounts;
        int targetBackRight = rearRightDriveMotor.getCurrentPosition() - moveCounts;

        // Apply power for leftward movement
        frontLeftDriveMotor.setPower(-power);
        frontRightDriveMotor.setPower(power);
        rearLeftDriveMotor.setPower(power);
        rearRightDriveMotor.setPower(-power);

        // Wait until the target encoder positions are reached
        while (isOpModeActive() &&
                frontLeftDriveMotor.getCurrentPosition() > targetFrontLeft &&
                frontRightDriveMotor.getCurrentPosition() < targetFrontRight &&
                rearLeftDriveMotor.getCurrentPosition() < targetBackLeft &&
                rearRightDriveMotor.getCurrentPosition() > targetBackRight) {
            // Continue moving left
        }

        // Stop the motors when the target is reached
        stopDriveMotors();

        linearOpMode.telemetry.addData("Status", "Finished Left Movement");
        linearOpMode.telemetry.update();
    }

    public void moveRight(double power, double distance) {
        int moveCounts = (int)(distance * COUNTS_PER_INCH);

        // Calculate target encoder positions for each wheel
        int targetFrontLeft = frontLeftDriveMotor.getCurrentPosition() + moveCounts;
        int targetFrontRight = frontRightDriveMotor.getCurrentPosition() - moveCounts;
        int targetBackLeft = rearLeftDriveMotor.getCurrentPosition() - moveCounts;
        int targetBackRight = rearRightDriveMotor.getCurrentPosition() + moveCounts;

        // Apply power for leftward movement
        frontLeftDriveMotor.setPower(power);
        frontRightDriveMotor.setPower(-power);
        rearLeftDriveMotor.setPower(-power);
        rearRightDriveMotor.setPower(power);

        // Wait until the target encoder positions are reached
        while (isOpModeActive() &&
                frontLeftDriveMotor.getCurrentPosition() < targetFrontLeft &&
                frontRightDriveMotor.getCurrentPosition() > targetFrontRight &&
                rearLeftDriveMotor.getCurrentPosition() > targetBackLeft &&
                rearRightDriveMotor.getCurrentPosition() < targetBackRight) {

            // Continue moving
        }

        // Stop the motors when the target is reached
        stopDriveMotors();

        linearOpMode.telemetry.addData("Status", "Finished Right Movement");
        linearOpMode.telemetry.update();
    }

    public void turnToHeading(double maxTurnSpeed, double heading)  {
        // Positive heading is right. Negative heading is left

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (isOpModeActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            //headingError = heading - getHeadingInDegrees();
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);
            linearOpMode.telemetry.addData("Robot Heading: ", getHeadingInDegrees());
            linearOpMode.telemetry.update();
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeadingInDegrees();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public double getHeading() {
        //YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        // YawPitchRollAngles orientation = expImu.getRobotYawPitchRollAngles();
        //return orientation.getYaw(AngleUnit.DEGREES);
        return getBotHeadingUsingIMU();
    }

    public void moveToTargetPosition(int targetFrontLeft, int targetFrontRight, int targetRearLeft, int targetRearRight){
        // Set Target FIRST, then turn on RUN_TO_POSITION
        frontLeftDriveMotor.setTargetPosition(targetFrontLeft);
        frontRightDriveMotor.setTargetPosition(targetFrontRight);
        rearLeftDriveMotor.setTargetPosition(targetRearLeft);
        rearRightDriveMotor.setTargetPosition(targetRearRight);

        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void initPropDetection(){
        visionProcessor.initPropDetection();
    }

    public int isPropDetected(){
        return visionProcessor.isPropDetected();
    }

    public void parkLeftOfBlueBackdrop(){
        sliderDrivePosition();
        moveLeft(DRIVE_SPEED,27.0);
    }

    public void parkRightOfBlueBackdrop(){
        sliderDrivePosition();
        moveRight(DRIVE_SPEED,22);
    }

    public void parkLeftOfRedBackdrop(){
        sliderDrivePosition();
        moveLeft(DRIVE_SPEED,18);
    }

    public void parkRightOfRedBackdrop(){
        sliderDrivePosition();
        moveRight(DRIVE_SPEED,38);
    }

    public void stopDriveMotors(){
        frontLeftDriveMotor.setPower(0);
        frontRightDriveMotor.setPower(0);
        rearLeftDriveMotor.setPower(0);
        rearRightDriveMotor.setPower(0);

        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setSliderIdlePosition() {
        linearSlide.setPower(IDLE_SPEED);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void sliderDrivePosition() {
        linearSlide.setTargetPosition(180);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(SLIDER_UP_SPEED);
        while (isOpModeActive() && linearSlide.isBusy()) {
            // wait
        }
        linearSlide.setPower(SLIDER_HOLD_SPEED);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void sliderReturnPosition() {
        linearSlide.setTargetPosition(100);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(SLIDER_UP_SPEED);
        linearSlide.setPower(SLIDER_HOLD_SPEED);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void sliderDrivePositionAuto() {
        linearSlide.setTargetPosition(90);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(SLIDER_UP_SPEED);
        while (isOpModeActive() && linearSlide.isBusy()) {
            // wait
        }
        linearSlide.setPower(SLIDER_HOLD_SPEED);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void sliderLine1Position(){
        linearSlide.setTargetPosition(1550);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(SLIDER_UP_SPEED);
        while (isOpModeActive() && linearSlide.isBusy()) {
            // wait
        }
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setSliderIdlePosition();
    }

    public void   sliderLowerThanLine1Position(){
        linearSlide.setTargetPosition(1350);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(SLIDER_UP_SPEED);
        while (isOpModeActive() && linearSlide.isBusy()) {
            // wait
        }
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setSliderIdlePosition();
    }

    public void sliderLine1PositionAndOuttake(){
        linearSlide.setTargetPosition(1550);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(SLIDER_UP_SPEED);
        while (isOpModeActive() && linearSlide.isBusy()) {
            // wait
        }
        outtake();
        linearOpMode.sleep(3000);
        intakeMotor.setPower(0);
        rightScoringServo.setPower(0);
        leftScoringServo.setPower(0);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setPower(SLIDER_HOLD_SPEED);
    }
    public void sliderDown() {
        linearSlide.setTargetPosition(0);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(0.4);
        while (isOpModeActive() && linearSlide.isBusy()) {
            // wait
        }
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setSliderIdlePosition();
    }
    public void drivePositionV2(){
        sliderDrivePosition();
        leftPlate.setPosition(1-(DELIVERY_PLATE_DRIVE_POS));
        rightPlate.setPosition(DELIVERY_PLATE_DRIVE_POS);
    }

    public void drivePositionV2Auto(){
        sliderDrivePositionAuto();
//        leftPlate.setPosition(1-(DELIVERY_PLATE_DRIVE_POS));
//        rightPlate.setPosition(DELIVERY_PLATE_DRIVE_POS);
    }

    public void moveSliderUp(){
        linearSlide.setPower(SLIDER_UP_SPEED);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveSliderDown(){
        linearSlide.setPower(-SLIDER_DOWN_SPEED);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void intake(){
        intakeMotor.setPower(-INTAKE_SPEED);
        rightScoringServo.setPower(-SERVO_SPEED);
        leftScoringServo.setPower(SERVO_SPEED);
    }
    public void intakeV2(){
        intakeMotor.setPower(-INTAKE_SPEED);
        leftPlate.setPosition(0.55);
        rightPlate.setPosition(0.45);
    }

    public void intakeV3(){
        intakeMotor.setPower(-INTAKE_SPEED);
        rightPlate.setPosition(DELIVERY_PLATE_INTAKE_POS);
        leftPlate.setPosition(1-DELIVERY_PLATE_INTAKE_POS);
    }

    public void setIntakeOuttakeIdle(){
        intakeMotor.setPower(IDLE_SPEED);
        rightScoringServo.setPower(IDLE_SPEED);
        leftScoringServo.setPower(IDLE_SPEED);
    }

    public void setIntakeIdle(){
        intakeMotor.setPower(IDLE_SPEED);
    }

    public void outtakeV2(){
        intakeMotor.setPower(INTAKE_SPEED);
        leftOutTakeV2();
        rightOutTakeV2();
    }
    public void outtake(){
        intakeMotor.setPower(INTAKE_SPEED);
        leftOutTake();
        rightOutTake();
    }

    public void deliveryV2(){
        delivery();
    }

    public void leftOutTake(){
        leftPlate.setPosition(180);
    }

    public void delivery() {
        sliderLowerThanLine1Position();
        linearOpMode.sleep(500);
        leftPlate.setPosition(1-DELIVERY_PLATE_SCORING_POS);
        rightPlate.setPosition(DELIVERY_PLATE_SCORING_POS);
        linearOpMode.sleep(1000);
    }

    public void leftOutTakeV2(){
        leftPlate.setPosition(0);
    }
    public void rightOutTakeV2(){
        rightPlate.setPosition(1);
    }

    public void leftOutTakeIdle(){
        leftScoringServo.setPower(0);
    }

    public void rightOutTake(){
        rightPlate.setPosition(0);}

    public void bumpSlider(){
        linearSlide.setPower(linearSlide.getPower()+0.2);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveHangerUp(){
        hangMotor.setPower(HANGING_MOTOR_UP_SPEED);
    }

    public void moveHangerDown(){
        hangMotor.setPower(-HANGING_MOTOR_DOWN_SPEED);
    }

    public void makeHangerIdle(){
        hangMotor.setPower(HANGING_MOTOR_IDLE_SPEED);
    }

    public void launchDrone(){
        droneLauncher.setPosition(-DRONE_LAUNCHER_SPEED);
    }

    private void initVisionProcessor(){
        visionProcessor = new VisionProcessor(linearOpMode);
    }

    private void initVisionProcessor(String alliance){
        visionProcessor = new VisionProcessor(linearOpMode, alliance);
    }

    private void initDriveMotors() {
        frontLeftDriveMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        rearLeftDriveMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightDriveMotor = hardwareMap.dcMotor.get("frontRightMotor");
        rearRightDriveMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Reverse the left side motors
        frontLeftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset the encoders and set the motors to BRAKE mode
        frontLeftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set the encoders for closed loop speed control, and reset the heading.
        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initIntakeMotor() {
        intakeMotor = hardwareMap.dcMotor.get("inMotor");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    private void initHangingMotor() {
        hangMotor = hardwareMap.dcMotor.get("hangMotor");
        hangMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    private void initServos() {
        rightScoringServo = hardwareMap.crservo.get("rightScoringServo");
        leftScoringServo = hardwareMap.crservo.get("leftScoringServo");
        rightScoringServo.setDirection(CRServo.Direction.FORWARD);
        leftScoringServo.setDirection(CRServo.Direction.FORWARD);
        droneLauncher = hardwareMap.servo.get("drone_launcher");
        leftPlate = hardwareMap.servo.get("leftPlate");
        rightPlate = hardwareMap.servo.get("rightPlate");
    }

    private void initLinearSlide() {
        linearSlide = hardwareMap.dcMotor.get("linearSlide1");
        linearSlide.setDirection(DcMotor.Direction.FORWARD);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

//    private void initIMU() {
//        imu = hardwareMap.get(IMU.class, "imu");
//        // Adjust the orientation parameters to match your robot
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
//        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//        imu.initialize(parameters);
//    }

//    public void initExpIMU() {
//        expImu = hardwareMap.get(BNO055IMU.class, "imu1");
//        // Adjust the orientation parameters to match your robot
//      /*  IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP));*/
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//        //parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        expImu.initialize(parameters);
//    }

    private void initExpIMU() {
        expImu = hardwareMap.get(IMU.class, "imu1");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        expImu.initialize(parameters);
    }

    public double getBotHeadingUsingIMU(){
        return expImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//        return expImu.getAngularOrientation().firstAngle;
    }
    public double getHeadingInDegrees() {
        return expImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//        return expImu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle;
    }

    public void resetIMU(){
        //imu.resetYaw();
    }

    public void resetExpIMU(){
        expImu.resetYaw();
    }

    public boolean isOpModeActive(){
        return linearOpMode.opModeIsActive();
    }
}