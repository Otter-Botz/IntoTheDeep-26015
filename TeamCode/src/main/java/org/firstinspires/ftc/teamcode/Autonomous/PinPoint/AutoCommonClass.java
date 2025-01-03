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
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;

    //Servo
    public Servo clawServo;
    public static Servo armSliderServo;
    public Servo wristServo;

    //Slide Motors
    public DcMotor sliderMotorMotor;
    public DcMotor sliderMotor;

    //PID_Arm Motor
    public DcMotor armMotor;

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

    private void initSliderReverse() {

        sliderMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        sliderMotorMotor = hardwareMap.get(DcMotor.class, "slideMotorMotor");

        sliderMotor.setDirection(DcMotor.Direction.FORWARD);
        sliderMotorMotor.setDirection(DcMotor.Direction.REVERSE);

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


    public void sliderUp(int position) {

        // for testing - start
        telemetry.addData("Motor Left Current", sliderMotor.getCurrentPosition());
        telemetry.addData("Motor Right Current", sliderMotorMotor.getCurrentPosition());
        telemetry.update();
        sleep(3000);
        // for testing - end

        // Previous Value
        sliderMotor.setTargetPosition(position);
        sliderMotorMotor.setTargetPosition(position);

        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power and let them move to the target
        sliderMotorMotor.setPower(0.2);  // Adjust power as needed
        sliderMotor.setPower(0.2);

        // Wait until motors reach their target
        runtime.reset();

        // for testing - start
        telemetry.addData("Motor Left Current New", sliderMotor.getCurrentPosition());
        telemetry.addData("Motor Right Current New", sliderMotorMotor.getCurrentPosition());
        telemetry.update();
        // for testing - end

        while (opModeIsActive() && runtime.seconds() < 0.85) {
            telemetry.addData("Motor Left Current Inside", sliderMotor.getCurrentPosition());
            telemetry.addData("Motor Right Current Inside", sliderMotorMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop the motors once target is reached
        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sliderMotor.setPower(0.01);
        //sliderMotorMotor.setPower(0.01);
    }

    public void moveSliders(int position, double power) {

        while(sliderMotor.getCurrentPosition() > position){
            sliderMotorMotor.setPower(power);  // Adjust power as needed
            sliderMotor.setPower(power);
        }

        if(sliderMotor.getCurrentPosition() > position) {
            sliderMotor.setPower(0);
            sliderMotorMotor.setPower(0);
        }
    }

    public void moveSlides(int targetPos, int targetPos2, int minPosition, int maxPosition, double power, LinearOpMode opMode) {

    }

    public void moveSlidersToPositionInRange(
            DcMotor slider1,
            DcMotor slider2,
            int targetPosition1,
            int targetPosition2,
            int minPosition,
            int maxPosition,
            double power,
            LinearOpMode opMode
    ) {

        // Constrain target positions to the specified range
        targetPosition1 = Math.max(minPosition, Math.min(maxPosition, targetPosition1));
        targetPosition2 = Math.max(minPosition, Math.min(maxPosition, targetPosition2));

        // Ensure sliders use encoders
        slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slider1.setTargetPosition(targetPosition1);
        slider2.setTargetPosition(targetPosition2);

        slider1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power to both sliders
        slider1.setPower(power);
        slider2.setPower(power);

        // Wait until both sliders reach their positions
        while (opMode.opModeIsActive() &&
                (slider1.isBusy() || slider2.isBusy())) {
            // Optionally, add telemetry here to display progress
            opMode.telemetry.addData("Slider1 Position", slider1.getCurrentPosition());
            opMode.telemetry.addData("Slider2 Position", slider2.getCurrentPosition());
            opMode.telemetry.update();
        }

        // Stop the sliders once target positions are reached
        slider1.setPower(0);
        slider2.setPower(0);

        // Reset the mode to avoid running unexpectedly in RUN_TO_POSITION
        slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void sliderDownElapsedTime() {
        timer.reset();

        // In your loop, stop the motor after a certain time
        if (timer.seconds() > 5.0) { // 5 seconds

            sliderMotor.setTargetPosition(-900);
            sliderMotorMotor.setTargetPosition(-900);

            sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            sliderMotor.setPower(0.2);
            sliderMotorMotor.setPower(0.2);

        }
    }

    public void sliderDown(int position) {

        initSliderReverse();

        // for testing - start
        telemetry.addData("Motor Left Current", sliderMotor.getCurrentPosition());
        telemetry.addData("Motor Right Current", sliderMotorMotor.getCurrentPosition());
        telemetry.update();
        sleep(3000);
        // for testing - end

        // Previous Value
        sliderMotor.setTargetPosition(position);
        sliderMotorMotor.setTargetPosition(position);

        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power and let them move to the target
        sliderMotorMotor.setPower(-0.2);  // Adjust power as needed
        sliderMotor.setPower(-0.2);

        // Wait until motors reach their target
        runtime.reset();

        // for testing - start
        telemetry.addData("Motor Left Current New", sliderMotor.getCurrentPosition());
        telemetry.addData("Motor Right Current New", sliderMotorMotor.getCurrentPosition());
        telemetry.update();
        // for testing - end

        while (opModeIsActive() && runtime.seconds() < 0.85) {
            telemetry.addData("Motor Left Current Inside", sliderMotor.getCurrentPosition());
            telemetry.addData("Motor Right Current Inside", sliderMotorMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop the motors once target is reached
        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sliderMotor.setPower(0.01);
        //sliderMotorMotor.setPower(0.01);

        initSlider();
    }

    public void scoreHighBasket(int position) {
        sliderUp(position);
        lowbasketslider();
    }

    public void mainSliderDown() {
        sliderMotor.setTargetPosition(2);
        sliderMotorMotor.setTargetPosition(2);

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

    private DcMotor slider1;
    private DcMotor slider2;

    // PID coefficients
    private static final double Kp = 0.01;
    private static final double Ki = 0.0;
    private static final double Kd = 0.0;

    // PID variables
    private double integral1 = 0;
    private double lastError1 = 0;
    private double integral2 = 0;
    private double lastError2 = 0;

    private void moveToPosition(int targetPosition1, int targetPosition2) {
        boolean slider1AtTarget = false;
        boolean slider2AtTarget = false;

        while (opModeIsActive() && (!slider1AtTarget || !slider2AtTarget)) {
            // Compute errors
            double error1 = targetPosition1 - slider1.getCurrentPosition();
            double error2 = targetPosition2 - slider2.getCurrentPosition();

            // Compute integral
            integral1 += error1;
            integral2 += error2;

            // Compute derivative
            double derivative1 = error1 - lastError1;
            double derivative2 = error2 - lastError2;

            // Update last error
            lastError1 = error1;
            lastError2 = error2;

            // Compute power using PID formula
            double power1 = (Kp * error1) + (Ki * integral1) + (Kd * derivative1);
            double power2 = (Kp * error2) + (Ki * integral2) + (Kd * derivative2);

            // Clamp power to prevent excessive values
            power1 = Math.max(-1.0, Math.min(1.0, power1));
            power2 = Math.max(-1.0, Math.min(1.0, power2));

            // Set motor powers
            slider1.setPower(power1);
            slider2.setPower(power2);

            // Check if sliders are at target position (with a tolerance)
            slider1AtTarget = Math.abs(error1) < 10; // Tolerance of 10 ticks
            slider2AtTarget = Math.abs(error2) < 10;

            // Telemetry for debugging
            telemetry.addData("Target Position 1", targetPosition1);
            telemetry.addData("Current Position 1", slider1.getCurrentPosition());
            telemetry.addData("Power 1", power1);
            telemetry.addData("Target Position 2", targetPosition2);
            telemetry.addData("Current Position 2", slider2.getCurrentPosition());
            telemetry.addData("Power 2", power2);
            telemetry.update();
        }

        // Stop motors
        slider1.setPower(0);
        slider2.setPower(0);
    }

    // PIDF coefficients
    private static final double KP = 1.0;
    private static final double KI = 0.0;
    private static final double KD = 0.1;
    private static final double KF = 0.05;

    // PIDF variables
    private double slider1Integral = 0;
    private double slider2Integral = 0;
    private double slider1PrevError = 0;
    private double slider2PrevError = 0;

    private ElapsedTime timer = new ElapsedTime();

    public void runOpMode() {
        // Initialize motors
        slider1 = hardwareMap.get(DcMotor.class, "slider1");
        slider2 = hardwareMap.get(DcMotor.class, "slider2");

        // Reset encoders
        slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to run using encoders
        slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the start of the OpMode
        timer.reset();

        // Move sliders to target positions in sequence
        moveSliders(1000, 500, 3.0);  // Example: Move to 1000 for slider1 and 500 for slider2 within 3 seconds
        moveSliders(0, 0, 2.0);       // Move both sliders back to their starting positions
    }

    private void moveSliders(int slider1Target, int slider2Target, double timeout) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < timeout) {
            // Get current positions
            int slider1Current = slider1.getCurrentPosition();
            int slider2Current = slider2.getCurrentPosition();

            // Simulated velocity (optional; replace with actual velocity measurement if available)
            double slider1Velocity = slider1.getPower();
            double slider2Velocity = slider2.getPower();

            // Compute PIDF outputs
            double slider1Power = calculatePIDF(slider1Target, slider1Current, slider1Velocity, true);
            double slider2Power = calculatePIDF(slider2Target, slider2Current, slider2Velocity, false);

            // Apply power to motors
            slider1.setPower(slider1Power);
            slider2.setPower(slider2Power);

            // Telemetry for debugging
            telemetry.addData("Slider 1 Position", slider1Current);
            telemetry.addData("Slider 1 Target", slider1Target);
            telemetry.addData("Slider 1 Power", slider1Power);
            telemetry.addData("Slider 2 Position", slider2Current);
            telemetry.addData("Slider 2 Target", slider2Target);
            telemetry.addData("Slider 2 Power", slider2Power);
            telemetry.update();

            // Stop if both sliders are within tolerance
            if (Math.abs(slider1Target - slider1Current) < 10 && Math.abs(slider2Target - slider2Current) < 10) {
                break;
            }
        }

        // Stop the motors after reaching target
        slider1.setPower(0);
        slider2.setPower(0);
    }

    private double calculatePIDF(int target, int current, double velocity, boolean isNormal) {
        double error = target - current;
        double deltaTime = timer.seconds();
        timer.reset();

        // Integral calculation
        if (isNormal) {
            slider1Integral += error * deltaTime;
        } else {
            slider2Integral += error * deltaTime;
        }

        // Derivative calculation
        double derivative;
        if (isNormal) {
            derivative = (error - slider1PrevError) / deltaTime;
            slider1PrevError = error;
        } else {
            derivative = (error - slider2PrevError) / deltaTime;
            slider2PrevError = error;
        }

        // PIDF formula
        double output = KP * error + KI * (isNormal ? slider1Integral : slider2Integral) +
                KD * derivative + KF * velocity;

        // Reverse logic for slider2
        return isNormal ? output : -output;
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

