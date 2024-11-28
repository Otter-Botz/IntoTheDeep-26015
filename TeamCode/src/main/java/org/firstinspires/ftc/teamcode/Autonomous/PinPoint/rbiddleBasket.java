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
    public void highBasket(){
        sliderMotor.setTargetPosition(900);
        sliderMotorMotor.setTargetPosition(900);
    }

    //PID_Arm
    private DcMotor armMotor;
    PID_Arm movements = new PID_Arm();
    public void downGrab(){
        armMotor.setTargetPosition(221);
    }


    // Claw/Wrist
    private Servo clawServo;
    private Servo wristServo;
    public void clawOpen(){
        clawServo.setPosition(0);
    }
    public void clawClose(){
        clawServo.setPosition(0.3);
    }
    public void wristUp(){
        wristServo.setPosition(0.6);
    }
    public void wristDown(){
        wristServo.setPosition(0.1);
    }



    IMU imu;



    @Override
    public void runOpMode() {
        // 22.8 Ticks per inch
        // Initialize for Auto
        initAuto();

        // Wait
        waitForStart();
        resetRuntime();

        //Positive X Forward
        //Negative X Backward

        // Drive to basket
        driveToPos(400, -200);
        gyroTurnToAngle(45);
        //Score
        //driveToPos(200,400);
        //gyroTurnToAngle(90);
        //Pick Up


        // Turn towards basket
        //gyroTurnToAngle(-90);

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

        while (opModeIsActive() && ((Math.abs(targetX - odo.getPosX()) > 50)
                || (Math.abs(targetY - odo.getPosY())) > 50)) {
            odo.update();

            double x = 0.001 * (targetX - odo.getPosX());
            double y = -0.001 * (targetY - odo.getPosY());

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


