package org.firstinspires.ftc.teamcode.Autonomous.PinPoint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.PID_Arm;
import org.firstinspires.ftc.teamcode.common.claw;
import org.firstinspires.ftc.teamcode.common.wrist;

@Autonomous
public class rbiddleSpecimen extends LinearOpMode {

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

    //PID_Arm
    private DcMotor armMotor;

    // Claw/Wrist
    private Servo clawServo;
    private Servo wristServo;

    AutoMechanisms mechanisms = new AutoMechanisms();
    ElapsedTime time = new ElapsedTime();
    PID_Arm arm = new PID_Arm();
    claw claw = new claw();
    wrist wrist = new wrist();

    double tickPerInch = 23;

    IMU imu;

    @Override
    public void runOpMode() {
        // 22.8 ticks per inch for bot
        //21.18

        // Initialize for Auto
        initAuto();

        // Wait
        waitForStart();
        resetRuntime();

        while (opModeInInit()) {
            telemetry.addData("yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        }


        arm.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        claw.set(claw.close);
        wrist.set(wrist.down);


        driveToPos(-tickPerInch * 32, -270);

        while (time.seconds() < 2) {
            arm.math(221);
        }

        wrist.set(wrist.up);
        boolean runOnce = true;
        while (time.seconds() > 1.5 && time.seconds() < 4 ) {
            if(runOnce) {
                arm.math(1250);
                sleep(1000);
                claw.set(claw.open);
                arm.math(1115);
                sleep(500);
                runOnce=false;
            }
            driveToPos(-tickPerInch * 22, tickPerInch * 30);
        }
        headingCorrect();
        telemetry.addData("yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();

        /*
        //Move Forward to push first sample
        driveToPos(-tickPerInch * 55, tickPerInch * 30);
        //move right to be infront of first sample
        driveToPos(-tickPerInch * 55, tickPerInch * 40);
        //push first sample to observation zone
        driveToPos(-tickPerInch * 15, tickPerInch * 40);
        //move forward to push second sample
        driveToPos(-tickPerInch * 50, tickPerInch * 40);
        //move right to be infront of second sample
        driveToPos(-tickPerInch * 50, tickPerInch * 49);
        //push second sample to observation zone
        driveToPos(-tickPerInch * 15, tickPerInch * 49);
        driveToPos(-tickPerInch * 50, tickPerInch * 49);

        //gyroTurnToAngle(0);

        //claw.set(claw.open);

        //sleep(500);


        //39

        /*




        sleep(1000);

        driveToPos(450,-(tickPerInch * 53));


        sleep(1000);
        mechanisms.clawClose();
        mechanisms.wristUp();
        mechanisms.backGrab();
        mechanisms.clawOpen();
        mechanisms.belowRung();
        sleep(2000);
        mechanisms.backGrab();
        mechanisms.clawClose();
        driveToPos(-1334,(tickPerInch * 13));
        mechanisms.belowRung();

        mechanisms.clawOpen();
        driveToPos(1334,-(tickPerInch * 13));
        mechanisms.backGrab();
        mechanisms.clawClose();
        driveToPos(-1334,(tickPerInch * 13));
        mechanisms.belowRung();

        mechanisms.clawOpen();
        mechanisms.backGrab();
        driveToPos(1403,(tickPerInch * 26));
        driveToPos(1403,-(tickPerInch * 44));
        */




    }

    public void driveToPos(double targetX, double targetY) {
        odo.update();
        boolean telemAdded = false;

        if (!telemAdded) {
            telemetry.addData("PosX()", odo.getPosX());
            telemetry.addData("PosY()", odo.getPosY());
            telemetry.addData("TargetX", targetX);
            telemetry.addData("TargetY", targetY);

            telemetry.update();
            telemAdded = true;
        }

        while (opModeIsActive() && ((Math.abs(targetX - odo.getPosX()) > 70)
                || (Math.abs(targetY - odo.getPosY())) > 70)) {
            odo.update();

            //Working
            double x = 0.0015 * (targetX - odo.getPosX());
            double y = -0.0015 * (targetY - odo.getPosY());

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

            if ((driveMotorsPower < 0.2) && (driveMotorsPower > 0)) {
                driveMotorsPower = 0.2;
            } else if ((driveMotorsPower > -0.2) && (driveMotorsPower < 0)) {
                driveMotorsPower = -0.2;
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
    public void headingCorrect() {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = 0 - heading;
        gyroTurnToAngle(error);
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

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Sliders
        sliderMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        sliderMotorMotor = hardwareMap.get(DcMotor.class, "slideMotorMotor");

        sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotorMotor.setDirection(DcMotor.Direction.REVERSE);

        // Claw/Wrist
        clawServo = hardwareMap.get(Servo.class,  "clawServo");
        wristServo = hardwareMap.get(Servo.class,  "wristServo");
        wrist.init(hardwareMap);
        claw.init(hardwareMap);
        arm.init(hardwareMap);
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
