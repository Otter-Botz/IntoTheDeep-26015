package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTest", group = "Servo")
public class ServoTest  extends LinearOpMode {




    @Override
    public void runOpMode() throws InterruptedException {
        CRServo servo1 = hardwareMap.crservo.get("servo1");
        Servo servo2 = hardwareMap.servo.get("servo2");
        waitForStart();


        while (opModeIsActive()) {
            /*
            if (gamepad1.a){
                servo1.setPower(1);
            }
            else if (gamepad1.b){
                servo1.setPower(-1);
            } else {
                servo1.setPower(0);
            }
            */



                if (gamepad1.a) {
                    servo2.setPosition(0.5);
                }
                else if (gamepad1.b) {
                    servo2.setPosition(0.6);
                }





        }

    }
}