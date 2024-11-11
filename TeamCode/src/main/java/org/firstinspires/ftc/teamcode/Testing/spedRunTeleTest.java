package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "adnanop")
public class spedRunTeleTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor slideMotorMotor = hardwareMap.dcMotor.get("slideMotorMotor");
        DcMotor slideMotor = hardwareMap.dcMotor.get("slideMotor");
        Servo claw = hardwareMap.servo.get("clawServo");
        Servo Wrist = hardwareMap.servo.get("wristServo");
        CRServo servoSlide = hardwareMap.crservo.get("servoSlide");
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {
            double slidePower = gamepad1.left_stick_y;
            slideMotorMotor.setPower(-slidePower);
            slideMotor.setPower(slidePower);
            servoSlide.setPower(gamepad1.right_stick_y);
            if (gamepad1.a) {
                claw.setPosition(1);
            } else if (gamepad1.b) {
                claw.setPosition(.35);
            }
        }
    }
}