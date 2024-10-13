package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "armTestLol")
public class armTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor armMotorUno = hardwareMap.dcMotor.get("armMotorUno");
        armMotorUno.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive()) {
            armMotorUno.setPower(gamepad1.left_stick_x);
        }
    }
}