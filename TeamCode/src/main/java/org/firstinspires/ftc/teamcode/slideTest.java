package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "slideTest", group = "otterbotz")
public class slideTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        DcMotor slideMotor = hardwareMap.dcMotor.get("linearSlide1");
        waitForStart();

        while (opModeIsActive()) {
          slideMotor.setPower(gamepad1.left_stick_y);


        }


    }
}
