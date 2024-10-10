package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ServoTest", group = "Servo")
public class sliderTest  extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor slideMotor1 = hardwareMap.dcMotor.get("slideMotor1");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a){
                slideMotor1.setPower(0.6);
            }
            if (gamepad1.b){
                slideMotor1.setPower(0);
            }
            if(gamepad1.x){
                slideMotor1.setPower(-0.6);
            }
//allah huakbar(this is so it works)
        }

    }
}
