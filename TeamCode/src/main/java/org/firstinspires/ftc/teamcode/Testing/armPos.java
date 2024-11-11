package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class armPos extends LinearOpMode {
    DcMotor armMotor;
    TouchSensor touchSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        touchSensor = hardwareMap.get(TouchSensor.class, "sensorTouch");

        waitForStart();
        while (opModeIsActive()) {
            if (touchSensor.isPressed()) {
                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            telemetry.addData("pos", armMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
