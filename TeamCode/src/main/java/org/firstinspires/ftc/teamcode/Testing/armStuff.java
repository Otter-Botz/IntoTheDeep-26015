package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
@TeleOp
public class armStuff extends LinearOpMode {

    public DcMotor armSlideMotor;
    public CRServo armServo;
    //int motorTicks = 125;
    //int motorPos = 150;
    //int encoderPosition = armSlideMotor.getCurrentPosition();
    public void runOpMode() throws InterruptedException {

waitForStart();
        armSlideMotor = hardwareMap.get(DcMotor.class, "armSlideMotor");


        while (opModeIsActive()) {

            telemetry.addData("pos", armSlideMotor.getCurrentPosition());
            telemetry.addData("pos3.1415926535897", armSlideMotor.getCurrentPosition());
            telemetry.update();
        }
    }


}
