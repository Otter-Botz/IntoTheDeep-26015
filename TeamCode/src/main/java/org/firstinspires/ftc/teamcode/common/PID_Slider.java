package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
public class PID_Slider {
    private static PIDController controller;
    public static double p = 0.00005, i = 0, d = 0.0001;
    public static double f = 0.01;
    public double target;

    private final double ticks_per_degree = 384.5 / 360;

    public static DcMotor sliderMotor;

    public PID_Slider(DcMotor motor, double initTarget) {
        sliderMotor = motor;
        controller = new PIDController(p, i, d);
        target = initTarget;
    }

    public void math() {
        controller.setPID(p, i, d);
        int armPos = sliderMotor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_per_degree)) * f;
        double power = pid + ff;
        sliderMotor.setPower(power);
    }

//    public void AutoUp() {
//        up();
//    }
//
//    public void up() {
//        target = 1350;
//    }
//
//    public void armRespond(double value) {
//        value = value * 1;
//        target = target - value;
//    }
//
//    public void down() {
//        target = 221;
//    }
//


//    public double getPosition() {
//        return sliderMotor.getCurrentPosition();
//    }


}










