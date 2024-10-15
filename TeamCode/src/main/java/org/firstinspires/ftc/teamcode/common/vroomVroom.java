package org.firstinspires.ftc.teamcode.common;



import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class vroomVroom  {
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    private IMU imu;





        public  void init(HardwareMap hwMap) {
            frontLeftMotor = hwMap.get(DcMotor.class, "leftFront");
            backLeftMotor = hwMap.get(DcMotor.class, "leftFront");
            frontRightMotor = hwMap.get(DcMotor.class, "leftBack");
            backRightMotor = hwMap.get(DcMotor.class, "rightBack");

            frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


             imu = hwMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);
        }

        public void resetYaw(){
                imu.resetYaw();

        }




        public void math(double y, double x, double rx, double power ){




            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            //get plane coordinates of robot from yaw
            double a = Math.cos(-botHeading);
            double b = Math.sin(-botHeading);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator * power;
            double backLeftPower = (rotY - rotX + rx) / denominator * power;
            double frontRightPower = (rotY - rotX - rx) / denominator * power;
            double backRightPower = (rotY + rotX - rx) / denominator * power;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

        }





        /*
        telemetry.addData("rotx", rotX);
        telemetry.addData("roty", rotY);
        telemetry.addData("frontleftpower", frontLeftPower);
        telemetry.addData("frontrightpower", frontRightPower);
        telemetry.addData("backleftpower", backLeftPower);
        telemetry.addData("backrightpower", backRightPower);
        telemetry.addData("Denominator", denominator);
        telemetry.addData("Parameterforimu", parameters);
        telemetry.addData("leftStickY", gamepad1.left_stick_y);
        telemetry.addData("leftStickX", gamepad1.left_stick_x);
        telemetry.addData("rightStickY", gamepad1.right_stick_y);
        telemetry.addData("rightstickx", gamepad1.right_stick_x);
        telemetry.addData("imuyawpitchroll", imu.getRobotYawPitchRollAngles());
        telemetry.addData("cos of radian", Math.cos(-botHeading));
        telemetry.addData("sin of radian", Math.sin(-botHeading));
        telemetry.addData("radiens yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        telemetry.addData("a", a);
        telemetry.addData("b", b);
        telemetry.update();
        */




}

