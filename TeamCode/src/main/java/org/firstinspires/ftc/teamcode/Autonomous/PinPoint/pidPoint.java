package org.firstinspires.ftc.teamcode.Autonomous.PinPoint;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class pidPoint extends LinearOpMode {


   DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
   DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "leftBack");
   DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
   DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "rightBack");

    PIDFController pidf = new PIDFController(0, 0, 0, 0);

    double output = pidf.calculate(
            frontLeftMotor.getCurrentPosition(), 100
    );
    double output1 = pidf.calculate(
            backLeftMotor.getCurrentPosition(), 100
    );
    double output2 = pidf.calculate(
            frontRightMotor.getCurrentPosition(), 100
    );
    double output3 = pidf.calculate(
            backRightMotor.getCurrentPosition(), 100
    );


    @Override
    public void runOpMode() throws InterruptedException {

        while (!pidf.atSetPoint()) {
                frontLeftMotor.setVelocity(output);
                backLeftMotor.setVelocity(output1);
                frontRightMotor.setVelocity(output2);
                backRightMotor.setVelocity(output3);
        }
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

    }
}
