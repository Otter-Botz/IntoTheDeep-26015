package org.firstinspires.ftc.teamcode.Autonomous.PIDAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CommonAuto {
    private static final double COUNTS_PER_MOTOR_REV = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    private static final double DRIVE_GEAR_REDUCTION = 1.0 ;     // No External Gearing.
    private static final double WHEEL_DIAMETER_INCHES = 4.0 ;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private final HardwareMap hardwareMap;
    private final LinearOpMode linearOpMode;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    private static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    private static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable
    private double          headingError  = 0;
    private double  targetHeading = 0;
    private double  turnSpeed     = 0;
    public CommonAuto(LinearOpMode callingLinearOpMode) {
        this.linearOpMode = callingLinearOpMode;
        this.hardwareMap = callingLinearOpMode.hardwareMap;
    }



    public void moveLeft(double power, double distance) {
        int moveCounts = (int) (distance * COUNTS_PER_INCH);

        // Calculate target encoder positions for each wheel
        int targetFrontLeft = frontLeftMotor.getCurrentPosition() - moveCounts;
        int targetFrontRight = frontRightMotor.getCurrentPosition() + moveCounts;
        int targetBackLeft = backLeftMotor.getCurrentPosition() + moveCounts;
        int targetBackRight = backRightMotor.getCurrentPosition() - moveCounts;

        // Apply power for leftward movement
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(-power);

        moveToTargetPosition( targetBackLeft, targetBackRight, targetFrontLeft, targetFrontRight);

    }
    public void moveRight(double power, double distance) {
        int moveCounts = (int)(distance * COUNTS_PER_INCH);

        // Calculate target encoder positions for each wheel
        int targetFrontLeft = frontLeftMotor.getCurrentPosition() + moveCounts;
        int targetFrontRight = frontRightMotor.getCurrentPosition() - moveCounts;
        int targetBackLeft = backLeftMotor.getCurrentPosition() - moveCounts;
        int targetBackRight = backRightMotor.getCurrentPosition() + moveCounts;

        // Apply power for rightward movement
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(-power);
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(power);

        moveToTargetPosition( targetBackLeft, targetBackRight, targetFrontLeft, targetFrontRight);


    }
    public void driveStraight(double power, double distance) {
        int moveCounts = (int) (distance * COUNTS_PER_INCH);

        // Calculate target encoder positions for each wheel
        int targetFrontLeft = frontLeftMotor.getCurrentPosition() + moveCounts;
        int targetFrontRight = frontRightMotor.getCurrentPosition() + moveCounts;
        int targetBackLeft = backLeftMotor.getCurrentPosition() + moveCounts;
        int targetBackRight = backRightMotor.getCurrentPosition() + moveCounts;

        // Apply Power for straight movement
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);

        moveToTargetPosition( targetBackLeft, targetBackRight, targetFrontLeft, targetFrontRight);

    }
    public void moveToTargetPosition(int targetFrontLeft, int targetFrontRight, int targetRearLeft, int targetRearRight){
        // Set Target FIRST, then turn on RUN_TO_POSITION
        frontLeftMotor.setTargetPosition(targetFrontLeft);
        frontRightMotor.setTargetPosition(targetFrontRight);
        backLeftMotor.setTargetPosition(targetRearLeft);
        backRightMotor.setTargetPosition(targetRearRight);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void moveBack(double power, double distance) {
        int moveCounts = (int) (distance * COUNTS_PER_INCH);

        //Calculate target encoder positions for each wheel
        int targetFrontLeft = frontLeftMotor.getCurrentPosition() - moveCounts;
        int targetFrontRight = frontRightMotor.getCurrentPosition() - moveCounts;
        int targetBackLeft = backLeftMotor.getCurrentPosition() - moveCounts;
        int targetBackRight = backRightMotor.getCurrentPosition() - moveCounts;

        // Apply Power for back movement
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(-power);
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(-power);

        moveToTargetPosition( targetBackLeft, targetBackRight, targetFrontLeft, targetFrontRight);

    }
}
