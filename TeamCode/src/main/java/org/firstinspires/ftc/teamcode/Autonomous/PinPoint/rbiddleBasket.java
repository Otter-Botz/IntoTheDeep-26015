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

@Autonomous(name = "0+4Blue")
public class rbiddleBasket extends LinearOpMode {

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
    AutoMechanisms mechanisms = new AutoMechanisms();

    ElapsedTime runtime = new ElapsedTime();

    claw claw = new claw();
    ArmSlider ArmSlider = new ArmSlider();
    PID_Arm PID_Arm = new PID_Arm();
    vroomVroom vroom = new vroomVroom();
    wrist wrist = new wrist();
    private LinearOpMode linearOpMode;
    IMU imu;

    int ticksPerInchForward = 23;
    int ticksPerInchSideways = 22;

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



            telemetry.update();


            sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sliderMotorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            sliderMotorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        }


        // Wait
        waitForStart();
        resetRuntime();

        int minRange = -100;    // Minimum position
        int maxRange = -1100; // Maximum position
        int currentPosition = sliderMotor.getCurrentPosition();
        double power = 0.3;

        //X = Y and Y = x
        //Make sure claw is able to hold sample
        odo.resetPosAndIMU();
        sleep(300);
        claw.AutoClose();
        wrist.set(wrist.down);
        // drive to basket
        driveToPos(-ticksPerInchForward * 10, ticksPerInchSideways * 10);
        sleep(300);
        driveToPos(-ticksPerInchForward * 20, ticksPerInchSideways * 10);
        gyroTurnToAngle(40);
        // 24.5 Barely Making It In Basket
        driveToPos(-ticksPerInchForward * 23,ticksPerInchSideways * 10);

        //Score 1
        lowbasketslider();
        //Slider Down
        armDown();
        headingCorrectBasket();
        //Move to first sample
        armDown();
        driveToPos(-ticksPerInchForward * 19.5, ticksPerInchSideways * 26.5);
        claw.AutoOpen();
        sleep(100);
        driveToPos(-ticksPerInchForward * 25, ticksPerInchSideways * 36);
        //Pick Up Sample
        armDown();
        sleep(200);
        claw.AutoClose();
        sleep(300);
        wrist.set(0.5);
        sleep(200);
        driveToPos(-ticksPerInchForward * 21, ticksPerInchSideways * 26.5);
        sleep(500);
        driveToPos(-ticksPerInchForward * 32,ticksPerInchSideways * 14);
        gyroTurnToAngle(-40);
        //Score 2
        lowbasketsliderwithoutwrist();
        sleep(100);
        //Slider Down
        armDown();
        //Move to second sample
        //26.5 Sometimes Work
        gyroTurnToAngle(45);
        driveToPos(-ticksPerInchForward * 30, ticksPerInchSideways * 37);
        //Pick Up Sample
        armDown();
        sleep(200);
        claw.AutoClose();
        sleep(300);
        wrist.set(0.5);
        sleep(200);



        // Y is y and X is X
        //driveToPos(-ticksPerInchForward * 20, ticksPerInchSideways * 31);

        //headingCorrectSample();
        //Pick Up Sample
//        wrist.set(wrist.AutoDown);
//        sleep(500);

        //Move to Basket
//        driveToPos(-ticksPerInchForward * 24, ticksPerInchSideways * 20);
//        driveToPos(-ticksPerInchForward * 37, ticksPerInchSideways * 20);
//
//        gyroTurnToAngle(40);
//        lowbasketslider();
//        sleep(1000);
//        armDown();

        //gyroTurnToAngle(45);
        //driveToPos(-ticksPerInchForward * 5, ticksPerInchSideways * 20);

        //driveToPos(-ticksPerInchForward * 10, ticksPerInchSideways * 30);

//

//       sliderDown();
//        driveToPos(-ticksPerInchForward * 20, ticksPerInchSideways * 30);
//        gyroTurnToAngle(45);
//        wrist.set(wrist.up);
//        sleep(500);
//        claw.AutoOpen();
//        sleep(600);
//        claw.AutoClose();
//
//        //Score 2
//        driveToPos(-ticksPerInchForward * 23.5, ticksPerInchSideways * 8);
//        gyroTurnToAngle(40);
//
//        //Move to second sample
//        driveToPos(-ticksPerInchForward * 20, ticksPerInchSideways * 20);
//        gyroTurnToAngle(45);

        //Move to first sample


        //sleep(500);


    }

    public void lowbasketslider() {
        runtime.reset();
        // Run tasks for the entire autonomous period
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
            PID_Arm.math(1115);
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
    public void highbasketslider(double maxRange) {

        int minPosition = -100;    // Minimum position
        int maxPosition = -1100; // Maximum position
        int currentPosition = sliderMotor.getCurrentPosition();
        double power = 0.3;
        while (!(currentPosition == maxRange)) {
           sliderMotor.setPower(power);
           sliderMotorMotor.setPower(power);
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
        sliderMotor.setTargetPosition(1100);
        sliderMotorMotor.setTargetPosition(1100);
        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotor.setPower(0.3);
        sliderMotorMotor.setPower(0.3);
        sleep(1000);

        runtime.reset();
        // Run tasks for the entire autonomous period
        while (runtime.seconds() < 1) {
            PID_Arm.math(1115);
        }
        wrist.set(wrist.up);
        sleep(400);
        claw.AutoOpen();
    }

//    public void armDown() {
//        runtime.reset();
//        while (runtime.seconds() < 3) {
//            PID_Arm.math(-200);
//        }
//    }

    public void sliderDown() {
        sliderMotor.setTargetPosition(-10);
        sliderMotorMotor.setTargetPosition(-10);
        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotor.setPower(0.5);
        sliderMotorMotor.setPower(0.5);
        sleep(1000);
        //armDown();
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

        org.firstinspires.ftc.teamcode.common.ArmSlider.init(hardwareMap);
        PID_Arm.init(hardwareMap);
        claw.init(hardwareMap);
        vroom.init(hardwareMap);
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


