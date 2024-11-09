package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTest", group = "Test")
public class ServoTest  extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //CRServo servo1 = hardwareMap.crservo.get("servo1");
        Servo servo2 = hardwareMap.servo.get("clawServo");

        waitForStart();
        while (opModeIsActive()) {
                if (gamepad2.a) {
                    servo2.setPosition(0);
                }

        }

    }
}
