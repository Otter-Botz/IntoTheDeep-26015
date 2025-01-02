package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "PIDF_SliderTest", group = "Linear Opmode")
public class PID_SliderTest extends LinearOpMode {

    private DcMotor slider1;
    private DcMotor slider2;

    // PID coefficients
    private static final double Kp = 0.01;
    private static final double Ki = 0.0;
    private static final double Kd = 0.0;

    // PID variables
    private double integral1 = 0;
    private double lastError1 = 0;
    private double integral2 = 0;
    private double lastError2 = 0;

    @Override
    public void runOpMode() {
        // Initialize motors
        slider1 = hardwareMap.get(DcMotor.class, "slider1");
        slider2 = hardwareMap.get(DcMotor.class, "slider2");

        // Reverse one of the motors
        slider2.setDirection(DcMotor.Direction.REVERSE);

        // Reset motor encoders
        slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (opModeIsActive()) {
            // Example target positions for autonomous
            int targetPosition1 = 1000;
            int targetPosition2 = 1000;

            moveToPosition(targetPosition1, targetPosition2);
        }
    }

    private void moveToPosition(int targetPosition1, int targetPosition2) {
        boolean slider1AtTarget = false;
        boolean slider2AtTarget = false;

        while (opModeIsActive() && (!slider1AtTarget || !slider2AtTarget)) {
            // Compute errors
            double error1 = targetPosition1 - slider1.getCurrentPosition();
            double error2 = targetPosition2 - slider2.getCurrentPosition();

            // Compute integral
            integral1 += error1;
            integral2 += error2;

            // Compute derivative
            double derivative1 = error1 - lastError1;
            double derivative2 = error2 - lastError2;

            // Update last error
            lastError1 = error1;
            lastError2 = error2;

            // Compute power using PID formula
            double power1 = (Kp * error1) + (Ki * integral1) + (Kd * derivative1);
            double power2 = (Kp * error2) + (Ki * integral2) + (Kd * derivative2);

            // Clamp power to prevent excessive values
            power1 = Math.max(-1.0, Math.min(1.0, power1));
            power2 = Math.max(-1.0, Math.min(1.0, power2));

            // Set motor powers
            slider1.setPower(power1);
            slider2.setPower(power2);

            // Check if sliders are at target position (with a tolerance)
            slider1AtTarget = Math.abs(error1) < 10; // Tolerance of 10 ticks
            slider2AtTarget = Math.abs(error2) < 10;

            // Telemetry for debugging
            telemetry.addData("Target Position 1", targetPosition1);
            telemetry.addData("Current Position 1", slider1.getCurrentPosition());
            telemetry.addData("Power 1", power1);
            telemetry.addData("Target Position 2", targetPosition2);
            telemetry.addData("Current Position 2", slider2.getCurrentPosition());
            telemetry.addData("Power 2", power2);
            telemetry.update();
        }

        // Stop motors
        slider1.setPower(0);
        slider2.setPower(0);
    }
}
