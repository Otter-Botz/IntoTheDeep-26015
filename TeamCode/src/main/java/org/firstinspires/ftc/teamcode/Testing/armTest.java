package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;


@TeleOp(name = "armTestLol")
public class armTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor armMotorUno = hardwareMap.dcMotor.get("armMotor");
        armMotorUno.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorUno.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorUno.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                armMotorUno.setTargetPosition(525);
                armMotorUno.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            telemetry.addData("pos", armMotorUno.getCurrentPosition());
            telemetry.update();
        }
    }
}