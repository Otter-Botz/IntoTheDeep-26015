package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class RobotController {
    private LinearOpMode linearOpMode;
    private HardwareMap hardwareMap;
    //private VisionProcessor visionProcessor;
   // public static final String BLUE_ALLIANCE = "BLUE_ALLIANCE";
   // public static final String RED_ALLIANCE = "RED_ALLIANCE";

    // Drive motors
    private DcMotor frontLeftDriveMotor;
    private DcMotor rearLeftDriveMotor;
    private DcMotor frontRightDriveMotor;
    private DcMotor rearRightDriveMotor;

    // Slider
    public static DcMotor linearSlide;
    public static final double SLIDER_UP_SPEED = 1.0;
    public static final double SLIDER_DOWN_SPEED = 0.5;
    public static final double SLIDER_HOLD_SPEED = 0.001;
    public static final double IDLE_SPEED = 0.0001;
    // IMU
    public IMU expImu;
    //    public BNO055IMU expImu;
    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();



    public RobotController(LinearOpMode callingLinearOpMode) {
        this.linearOpMode = callingLinearOpMode;
        this.hardwareMap = callingLinearOpMode.hardwareMap;
    }

    public void initTeleOp(){
        initDriveMotors();
        initLinearSlide();
        initIMU();

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



    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */

    public static void setSliderIdlePosition() {
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

    private boolean isOpModeActive() {
        return false;
    }

    public void sliderReturnPosition() {
        linearSlide.setTargetPosition(100);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(SLIDER_UP_SPEED);
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

    public static void moveSliderUp(){
        linearSlide.setPower(SLIDER_UP_SPEED);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void moveSliderDown(){
        linearSlide.setPower(-SLIDER_DOWN_SPEED);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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


        private void initLinearSlide() {
        linearSlide = hardwareMap.dcMotor.get("linearSlide1");
        linearSlide.setDirection(DcMotor.Direction.FORWARD);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initIMU() {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }
    public void resetIMU(){
        //imu.resetYaw();
    }
        }
