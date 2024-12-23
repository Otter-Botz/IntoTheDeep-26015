package org.firstinspires.ftc.teamcode.Autonomous.PinPoint;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import android.net.wifi.p2p.nsd.WifiP2pDnsSdServiceRequest;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AutoCommonClass implements autoCommonInterface {
    private final LinearOpMode linearOpMode;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    //Drive Motors
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    //Servo
    private Servo clawServo;
    static Servo armSliderServo;
    private Servo wristServo;

    //Slide Motors
    DcMotor sliderMotorMotor;
    DcMotor sliderMotor;

    //PID_Arm Motor
    DcMotor armMotor;

    // Odo Pods and IMU
    GoBildaPinpointDriver odo;
    IMU imu;

    //Elapsed Time
    ElapsedTime runtime = new ElapsedTime();

    //Calling Class
    public AutoCommonClass(LinearOpMode callingLinearOpMode) {
        this.linearOpMode = callingLinearOpMode;
        this.hardwareMap = callingLinearOpMode.hardwareMap;
        this.telemetry = callingLinearOpMode.telemetry;
    }

    //Init
    private void initSlider() {

        sliderMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        sliderMotorMotor = hardwareMap.get(DcMotor.class, "slideMotorMotor");

        sliderMotor.setDirection(DcMotor.Direction.REVERSE);
        sliderMotorMotor.setDirection(DcMotor.Direction.FORWARD);

        sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    private void initArmMotor() {
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        controller = new PIDController(p, i, d);
    }


    private void initDriveMotors() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void initServo() {
        // Claw/Wrist
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");

        //Arm Slider
        armSliderServo = hardwareMap.get(Servo.class, "servoSlide");
    }

    private void initImu() {
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();
    }

    private void initPinPoint() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(150, -370); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
    }

    //Init Full Auto

    public void initAuto() {

        initImu();
        initDriveMotors();
        initServo();
        initSlider();
        initArmMotor();
        initPinPoint();
    }

    //Servo Positions

    //Claw Positions
    public double ClawOpen = 0.25;
    public double ClawClose = 0;

    //Wrist Positions
    public double WristUp = 0.25;
    public double WristSubmersible = 0.5;
    public double WristDown = 0;
    public double WristMiddle = 0.25;

    //Arm Slider Positions
    public double ArmSliderOut = 1;
    public double ArmSliderIn = 0;
    public double ArmSliderMiddle = 0.5;


    @Override
    public void set(double position) {
        wristServo.setPosition(position);
        clawServo.setPosition(position);
    }

    //Main Slider Positions/Math
    private int minRange = 5;
    private int maxRange = -10;

    //Adjust Positions

    public void MainSliderPositionChange() {
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


    public void SliderMath() {
        // logic: Set target positions for motors
        int targetPosition = (minRange + maxRange) / 2; // Midpoint of range
    }


    public void sliderUp() {
        sliderMotor.setTargetPosition(12);
        sliderMotorMotor.setTargetPosition(12);

        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power and let them move to the target
        sliderMotorMotor.setPower(0.2);  // Adjust power as needed
        sliderMotor.setPower(0.2);

        // Wait until motors reach their target
        runtime.reset();
        while (opModeIsActive() && (sliderMotor.isBusy() || sliderMotorMotor.isBusy()) && runtime.seconds() < 3) {
            telemetry.addData("Motor Left Current", sliderMotor.getCurrentPosition());
            telemetry.addData("Motor Right Current", sliderMotorMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop the motors once target is reached
        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMotor.setPower(0.01);
        sliderMotorMotor.setPower(0.01);
    }


    public void scoreHighBasket() {
        sliderUp();
        lowbasketslider();
    }

    public void mainSliderDown() {
        sliderMotor.setTargetPosition(0);
        sliderMotorMotor.setTargetPosition(0);

        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power and let them move to the target
        sliderMotorMotor.setPower(0.2);  // Adjust power as needed
        sliderMotor.setPower(0.2);

        // Wait until motors reach their target
        runtime.reset();
        while (opModeIsActive() && (sliderMotor.isBusy() || sliderMotorMotor.isBusy()) && runtime.seconds() < 3) {
            telemetry.addData("Motor Left Current", sliderMotor.getCurrentPosition());
            telemetry.addData("Motor Right Current", sliderMotorMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop the motors once target is reached
        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMotor.setPower(0.01);
        sliderMotorMotor.setPower(0.01);
    }

    public void armRestPosition() {
        runtime.reset();
        // Run tasks for the entire autonomous period
        while (runtime.seconds() < 1) {
            AutoPIDArmmath(950);
        }
    }

    public void slidersDown() {
        mainSliderDown();
        armRestPosition();
    }



    //PID_Arm

    //Positions
    private static PIDController controller;
    //p = 0.005 i = 0 d = 0.0001 f=0.01
    private static final double p = 0.005;
    private static final double i = 0;
    private static final double d = 0.0001;
    private static final double f = 0.01;
    private final double target = 100;
    private final double ticks_in_degrees = 2786.2 / 360;

    //Math

    public void AutoPIDArmmath(double target) {
        controller.setPID(p, i, d);
        int armPos = armMotor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
        double power = pid + ff;
        armMotor.setPower(power);
        // telemetry.addData("pos", slidePos);
        // telemetry.addData("target", target);
        // telemetry.update();
    }


    public void armDown() {
        runtime.reset();
        // Run tasks for the entire autonomous period
        while (runtime.seconds() < 1) {
            AutoPIDArmmath(0);
        }
    }

    public void armDownSliderOut() {
        runtime.reset();
        // Run tasks for the entire autonomous period
        while (runtime.seconds() < 1) {
            AutoPIDArmmath(200);
        }
        armSliderServo.setPosition(ArmSliderOut);
        sleep(300);
        clawServo.setPosition(ClawOpen);
        sleep(600);
        clawServo.setPosition(ClawClose);
        sleep(200);
        wristServo.setPosition(WristUp);
    }

    public void armUpSliderIn() {
        runtime.reset();
        // Run tasks for the entire autonomous period
        while (runtime.seconds() < 1) {
            AutoPIDArmmath(950);
        }
        armSliderServo.setPosition(ArmSliderIn);
    }


    public void lowbasketslider() {
        runtime.reset();
        while (runtime.seconds() < 1.75) {
            AutoPIDArmmath(1221  );
        }
        wristServo.setPosition(WristUp);
        sleep(200);
        clawServo.setPosition(ClawOpen);
    }


    public void lowbasketsliderwithoutwrist() {
        runtime.reset();
        // Run tasks for the entire autonomous period
        while (runtime.seconds() < 1.75) {
            AutoPIDArmmath(1221);
        }
        sleep(200);
        clawServo.setPosition(ClawOpen);
    }


    //PinPoint Movement


    public void driveToPos(double targetX, double targetY) {
        odo.update();
        boolean telemAdded = false;

        if (!telemAdded) {
            telemetry.addData("PosX()", odo.getPosX());
            telemetry.addData("PosY()", odo.getPosY());
            telemetry.addData("TargetX", targetX);
            telemetry.addData("TargetY", targetY);
            // telemetry.addData("pos", slidePos);
            // telemetry.addData("target", target);
            telemetry.update();
            telemAdded = true;
        }

        while (opModeIsActive() && ((Math.abs(targetX - odo.getPosX()) > 50)
                || (Math.abs(targetY - odo.getPosY())) > 50)) {
            odo.update();

            double x = 0.0017 * (targetX - odo.getPosX());
            double y = -0.0017 * (targetY - odo.getPosY());

            double botHeading = odo.getHeading();

            double rotY = y * Math.cos(-botHeading) - x * Math.sin(-botHeading);
            double rotX = y * Math.sin(-botHeading) + x * Math.cos(-botHeading);

            if (!telemAdded) {
                telemetry.addData("x: ", x);
                telemetry.addData("y: ", y);
                telemetry.addData("rotX: ", rotX);
                telemetry.addData("rotY: ", rotY);
                telemetry.update();
                telemAdded = true;
            }

            if (Math.abs(rotX) < 0.15) {
                rotX = Math.signum(rotX) * 0.15;
            }

            if (Math.abs(rotY) < 0.15) {
                rotY = Math.signum(rotY) * 0.15;
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
            double frontLeftPower = (rotX + rotY) / denominator;
            double backLeftPower = (rotX - rotY) / denominator;
            double frontRightPower = (rotX - rotY) / denominator;
            double backRightPower = (rotX + rotY) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            telemetry.addData("X: ", odo.getPosX());
            telemetry.addData("Y: ", odo.getPosY());
            telemetry.addData("Heading Odo: ", Math.toDegrees(odo.getHeading()));
            telemetry.addData("Heading IMU: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    //Turn

    public void gyroTurnToAngle(double turnAngle) {
        double error, currentHeadingAngle, driveMotorsPower;
        imu.resetYaw();

        error = turnAngle;

        while (opModeIsActive() && ((error > 1) || (error < -1))) {
            odo.update();
            telemetry.addData("X: ", odo.getPosX());
            telemetry.addData("Y: ", odo.getPosY());
            telemetry.addData("Heading Odo: ", Math.toDegrees(odo.getHeading()));
            telemetry.addData("Heading IMU: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();

            driveMotorsPower = error / 200;

            if ((driveMotorsPower < 0.3) && (driveMotorsPower > 0)) {
                driveMotorsPower = 0.3;
            } else if ((driveMotorsPower > -0.3) && (driveMotorsPower < 0)) {
                driveMotorsPower = -0.3;
            }

            // Positive power causes left turn
            frontLeftMotor.setPower(-driveMotorsPower);
            backLeftMotor.setPower(-driveMotorsPower);
            frontRightMotor.setPower(driveMotorsPower);
            backRightMotor.setPower(driveMotorsPower);

            currentHeadingAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = turnAngle - currentHeadingAngle;
        }
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    private boolean opModeIsActive() {
        return linearOpMode.opModeIsActive();
    }

    //Heading Correct
    public void headingCorrectBasket() {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = 84 - heading;
        gyroTurnToAngle(error);
    }

    public void headingCorrectSample() {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = 80 - heading;
        gyroTurnToAngle(error);
    }

}

