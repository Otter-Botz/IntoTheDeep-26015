package org.firstinspires.ftc.teamcode.Testing;

import android.text.method.Touch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;
@TeleOp
public class touchTest extends LinearOpMode{
    TouchSensor touchSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        touchSensor = hardwareMap.get(TouchSensor.class, "sensorTouch");
        waitForStart();
        while (opModeIsActive()) {
             if (touchSensor.isPressed()) {

                telemetry.addData("TRUE FACT", "RYAB IS SHORT");

            } else {
                telemetry.addData("FALSE FACT", "RYAB IS TALL");
            }
            telemetry.update();
        }
    }
}
