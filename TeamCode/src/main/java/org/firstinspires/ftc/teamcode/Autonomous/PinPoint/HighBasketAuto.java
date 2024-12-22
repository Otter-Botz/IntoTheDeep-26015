package org.firstinspires.ftc.teamcode.Autonomous.PinPoint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.ArmSlider;
import org.firstinspires.ftc.teamcode.common.PID_Arm;
import org.firstinspires.ftc.teamcode.common.claw;
import org.firstinspires.ftc.teamcode.common.vroomVroom;
import org.firstinspires.ftc.teamcode.common.wrist;

@Autonomous(name = "HighBasket")
public class HighBasketAuto extends LinearOpMode {

    // Odo Pods and IMU
    GoBildaPinpointDriver odo;

    // Drive Motors
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    // Sliders
    private DcMotor sliderMotor;
    private DcMotor sliderMotorMotor;

    // Claw/Wrist
    private Servo clawServo;
    private Servo wristServo;
    boolean pathRunning;
    //Arm
    public DcMotor armMotor;

    // Common Class


    ElapsedTime runtime = new ElapsedTime();

    claw claw = new claw();
    ArmSlider armSlider = new ArmSlider();
    PID_Arm PID_Arm = new PID_Arm();
    vroomVroom vroom = new vroomVroom();
    wrist wrist = new wrist();
    private LinearOpMode linearOpMode;
    IMU imu;

    int ticksPerInchForward = 23;
    int ticksPerInchSideways = 22;


    //Sliders
    private int minRange = 5;
    private int maxRange = -10;


    // logic: Set target positions for motors
    int targetPosition = (minRange + maxRange) /2; // Midpoint of range



    @Override
    public void runOpMode() throws InterruptedException {
        initAuto();


        while (opModeInInit()) {
            telemetry.addData("pos motor1", sliderMotor.getCurrentPosition());
            telemetry.addData("pos motor2", sliderMotorMotor.getCurrentPosition());
            telemetry.addData("pos motor1 target", sliderMotor.getTargetPosition());
            telemetry.addData("pos motor2 target", sliderMotorMotor.getTargetPosition());

            telemetry.addData("PosX()", odo.getPosX());
            telemetry.addData("PosY()", odo.getPosY());

            telemetry.addData("yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

            // Display telemetry for driver adjustment
            telemetry.addLine("Adjust using gamepad:");
            telemetry.addData("Min Range", minRange + " ticks (D-Pad Up/Down)");
            telemetry.addData("Max Range", maxRange + " ticks (D-Pad Left/Right)");


            telemetry.update();


            sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sliderMotorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            sliderMotorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        odo.resetPosAndIMU();
        sleep(300);
        claw.AutoClose();
        wrist.set(wrist.up);
        sleep(100);
        armSlider.set(armSlider.in);


        // drive to basket
//        driveToPos(-ticksPerInchForward * 10, ticksPerInchSideways * 10);
//        sleep(300);
//        driveToPos(-ticksPerInchForward * 20, ticksPerInchSideways * 10);
//        gyroTurnToAngle(40);
//        // 24.5 Barely Making It In Basket
//        driveToPos(-ticksPerInchForward * 24.5,ticksPerInchSideways * 8);

        //Score 1
        sliderUp();
//        sleep(500);
//        //Slider Down
//        armDown();
//        headingCorrectBasket();
//        wrist.set(wrist.up);

        //Park
//        armDown();
//        driveToPos(-ticksPerInchForward * 7, ticksPerInchSideways * 40);
//        sleep(300);
//        driveToPos(-ticksPerInchForward * 3, ticksPerInchSideways * 50);
//        sleep(300);
//        driveToPos(-ticksPerInchForward * 3, ticksPerInchSideways * 60);
//        sleep(1000);

    }

    public void lowbasketslider() {
        runtime.reset();
        while (runtime.seconds() < 1.75) {
            PID_Arm.math(1180);
        }
        wrist.set(wrist.AutoUp);
        sleep(200);
        claw.AutoOpen();
    }

    public void lowbasketsliderwithoutwrist() {
        runtime.reset();
        // Run tasks for the entire autonomous period
        while (runtime.seconds() < 1.75) {
            PID_Arm.math(1221);
        }
        sleep(200);
        claw.AutoOpen();
    }

    public void  armDown() {
        runtime.reset();
        // Run tasks for the entire autonomous period
        while (runtime.seconds() < 1 ) {
            PID_Arm.math(0);
        }

    }

    public void  headingCorrectBasket() {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = 84 - heading;
        gyroTurnToAngle(error);
    }

    public void  headingCorrectSample() {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = 80 - heading;
        gyroTurnToAngle(error);
    }

    public void scoreHighBasket() {
        sliderMotor.setTargetPosition(-10);
        sliderMotorMotor.setTargetPosition(-10);
        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotor.setPower(0.3);
        sliderMotorMotor.setPower(0.3);
        sleep(1000);
    }

//    public void armDown() {
//        runtime.reset();
//        while (runtime.seconds() < 3) {
//            PID_Arm.math(-200);
//        }
//    }


    public void sliderUp() {
        sliderMotor.setTargetPosition(-7);
        sliderMotorMotor.setTargetPosition(-7);

        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power and let them move to the target
        sliderMotorMotor.setPower(0.2);  // Adjust power as needed
        sliderMotor.setPower(0.2);

        // Wait until motors reach their target
        runtime.reset();
        while (opModeIsActive() && (sliderMotor.isBusy() || sliderMotorMotor.isBusy()) && runtime.seconds() < 3) {
            telemetry.addData("Target Position", targetPosition + " ticks");
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

    public void initAuto() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(150, -370); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        //Drive Motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");

        // Sliders
        sliderMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        sliderMotorMotor = hardwareMap.get(DcMotor.class, "slideMotorMotor");

        sliderMotor.setDirection(DcMotor.Direction.REVERSE);
        sliderMotorMotor.setDirection(DcMotor.Direction.FORWARD);


        sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Claw/Wrist
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");

        //org.firstinspires.ftc.teamcode.common.ArmSlider.init(hardwareMap);
        PID_Arm.init(hardwareMap);
        claw.init(hardwareMap);
        vroom.init(hardwareMap);
        armSlider.init(hardwareMap);
        //Slider.init(hardwareMap);
        wrist.init(hardwareMap);
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

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

}


