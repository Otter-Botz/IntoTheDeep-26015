package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoTest  extends LinearOpMode {


    CRServo servo2 = hardwareMap.crservo.get("servo1");

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();


        while (opModeIsActive()) {
            if (gamepad1.a){
                servo2.setPower(1);
            }

        }

    }
}
