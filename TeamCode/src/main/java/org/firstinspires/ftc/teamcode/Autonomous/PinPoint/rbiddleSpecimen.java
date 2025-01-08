package org.firstinspires.ftc.teamcode.Autonomous.PinPoint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.PID_Arm;

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
        while (opModeIsActive()){
            //mechanisms.math();
        }
        resetRuntime();


        driveToPos(600, 100);
        //mechanisms.aboveRung();
        driveToPos(450, -(tickPerInch * 42));
       /// mechanisms.downGrab();
        sleep(1000);
        driveToPos(450,-(tickPerInch * 53));
       // mechanisms.downGrab();
        sleep(1000);
        sleep(2000);
        driveToPos(-1334,(tickPerInch * 13));
        //mechanisms.aboveRung();
        driveToPos(1334,-(tickPerInch * 13));

        driveToPos(-1334,(tickPerInch * 13));

        //mechanisms.aboveRung();

        driveToPos(1403,(tickPerInch * 26));
        driveToPos(1403,-(tickPerInch * 44));



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

        while (opModeIsActive() && ((Math.abs(targetX - odo.getPosX()) > 30)
                || (Math.abs(targetY - odo.getPosY())) > 30)) {
            odo.update();

            //Working
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
