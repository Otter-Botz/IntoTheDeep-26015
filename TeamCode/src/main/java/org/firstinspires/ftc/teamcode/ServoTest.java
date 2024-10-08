package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "ServoTest", group = "Servo")
public class ServoTest  extends LinearOpMode {




    @Override
    public void runOpMode() throws InterruptedException {
        CRServo servo2 = hardwareMap.crservo.get("servo1");
        waitForStart();


        while (opModeIsActive()) {
            if (gamepad1.a){
                servo2.setPower(1);
            }
            else if (gamepad1.b){
                servo2.setPower(-1);
            } else {
                servo2.setPower(0);
            }

        }

    }
}
