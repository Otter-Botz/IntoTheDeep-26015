package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "slideTest", group = "otterbotz")
public class slideTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        DcMotor slideMotor = hardwareMap.dcMotor.get("slideMotor");
        DcMotor slideMotorMotor = hardwareMap.dcMotor.get("slideMotorMotor");



        waitForStart();
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive()) {
          telemetry.addData("pos", slideMotor.getCurrentPosition());
          telemetry.addData("pos", slideMotorMotor.getCurrentPosition());
          telemetry.update();

        }


    }
}
