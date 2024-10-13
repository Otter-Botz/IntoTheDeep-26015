package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
@TeleOp(name = "Arm")
public class ArmSlider extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo armSliderServo = hardwareMap.crservo.get("armSliderServo");
        waitForStart();
        while (opModeIsActive()){
            armSliderServo.setPower(gamepad1.left_stick_x);
        }
    }
}
//ki7,7ik867
//hirhdrigijdfbg