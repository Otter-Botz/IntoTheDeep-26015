package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "PIDF_SliderTest2", group = "Linear Opmode")
public class PIDF_SliderTest2 extends LinearOpMode {

    private DcMotor slider1;
    private DcMotor slider2;

    // PIDF coefficients
    private static final double KP = 1.0;
    private static final double KI = 0.0;
    private static final double KD = 0.1;
    private static final double KF = 0.05;

    // PIDF variables
    private double slider1Integral = 0;
    private double slider2Integral = 0;
    private double slider1PrevError = 0;
    private double slider2PrevError = 0;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize motors
        slider1 = hardwareMap.get(DcMotor.class, "slider1");
        slider2 = hardwareMap.get(DcMotor.class, "slider2");

        // Reset encoders
        slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to run using encoders
        slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the start of the OpMode
        waitForStart();
        timer.reset();

        // Move sliders to target positions in sequence
        moveSliders(1000, 500, 3.0);  // Example: Move to 1000 for slider1 and 500 for slider2 within 3 seconds
        moveSliders(0, 0, 2.0);       // Move both sliders back to their starting positions
    }

    private void moveSliders(int slider1Target, int slider2Target, double timeout) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < timeout) {
            // Get current positions
            int slider1Current = slider1.getCurrentPosition();
            int slider2Current = slider2.getCurrentPosition();

            // Simulated velocity (optional; replace with actual velocity measurement if available)
            double slider1Velocity = slider1.getPower();
            double slider2Velocity = slider2.getPower();

            // Compute PIDF outputs
            double slider1Power = calculatePIDF(slider1Target, slider1Current, slider1Velocity, true);
            double slider2Power = calculatePIDF(slider2Target, slider2Current, slider2Velocity, false);

            // Apply power to motors
            slider1.setPower(slider1Power);
            slider2.setPower(slider2Power);

            // Telemetry for debugging
            telemetry.addData("Slider 1 Position", slider1Current);
            telemetry.addData("Slider 1 Target", slider1Target);
            telemetry.addData("Slider 1 Power", slider1Power);
            telemetry.addData("Slider 2 Position", slider2Current);
            telemetry.addData("Slider 2 Target", slider2Target);
            telemetry.addData("Slider 2 Power", slider2Power);
            telemetry.update();

            // Stop if both sliders are within tolerance
            if (Math.abs(slider1Target - slider1Current) < 10 && Math.abs(slider2Target - slider2Current) < 10) {
                break;
            }
        }

        // Stop the motors after reaching target
        slider1.setPower(0);
        slider2.setPower(0);
    }

    private double calculatePIDF(int target, int current, double velocity, boolean isNormal) {
        double error = target - current;
        double deltaTime = timer.seconds();
        timer.reset();

        // Integral calculation
        if (isNormal) {
            slider1Integral += error * deltaTime;
        } else {
            slider2Integral += error * deltaTime;
        }

        // Derivative calculation
        double derivative;
        if (isNormal) {
            derivative = (error - slider1PrevError) / deltaTime;
            slider1PrevError = error;
        } else {
            derivative = (error - slider2PrevError) / deltaTime;
            slider2PrevError = error;
        }

        // PIDF formula
        double output = KP * error + KI * (isNormal ? slider1Integral : slider2Integral) +
                KD * derivative + KF * velocity;

        // Reverse logic for slider2
        return isNormal ? output : -output;
    }
}
