package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class armPos extends LinearOpMode {
    DcMotor armMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("pos", armMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
