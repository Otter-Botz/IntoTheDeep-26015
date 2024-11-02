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
        armMotorUno.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        waitForStart();
        while (opModeIsActive()) {
            armMotorUno.setPower(gamepad2.right_trigger);
            armMotorUno.setPower(-gamepad2.left_trigger);
            telemetry.addData("pos", armMotorUno.getCurrentPosition());
            telemetry.update();
        }
    }
}