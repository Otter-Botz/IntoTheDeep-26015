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
        Servo clawServo = hardwareMap.servo.get("clawServo");
        Servo wristServo = hardwareMap.servo.get("wristServo");

        waitForStart();
        while (opModeIsActive()) {
                //Zero claw position
                if (gamepad2.left_bumper) {
                    clawServo.setPosition(0);
                }
                //Zero wrist position
                else if (gamepad2.right_bumper) {
                    wristServo.setPosition(0);
                }
                //Wrist 0.25 Position
                else if (gamepad2.dpad_down) {
                    wristServo.setPosition(0.25);
                }
                //Wrist 0.5 Position
                else if (gamepad2.dpad_right) {
                    wristServo.setPosition(0.5);
                }
                //Wrist 0.75 Position
                else if (gamepad2.dpad_up) {
                    wristServo.setPosition(0.75);
                }
                //Wrist 1 Position
                else if (gamepad2.dpad_left) {
                    wristServo.setPosition(1);
                }
                //Claw 0.25 Position
                else if (gamepad2.a) {
                    clawServo.setPosition(0.25);
                }
                //Claw 0.5 Position
                else if (gamepad2.b) {
                    clawServo.setPosition(0.5);
                }
                //Claw 0.75 Position
                else if (gamepad2.y) {
                    clawServo.setPosition(0.75);
                }
                //Claw 1 Position
                else if (gamepad2.x) {
                    clawServo.setPosition(1);
                }
                else if (gamepad2.options) {
                    telemetry.addData("clawCurrentPosition", clawServo.getPosition());
                    telemetry.addData("wristCurrentPosition", wristServo.getPosition());
                    telemetry.update();
                }



        }

    }
}
