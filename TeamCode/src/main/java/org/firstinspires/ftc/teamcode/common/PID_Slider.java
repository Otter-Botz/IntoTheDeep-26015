package org.firstinspires.ftc.teamcode.common;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PID_Slider {
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degrees = 700/180;

    private DcMotor sliderMotorMotor;
    private DcMotor sliderMotor;

    public void math() {

        controller.setPID(p, i , d);
        int slidePos = sliderMotor.getCurrentPosition();
        int slidePos1 = sliderMotorMotor.getCurrentPosition();
        double pid = controller.calculate(slidePos, target);
        double pid1 = controller.calculate(slidePos1, target);
        double ff = Math.cos(Math.toRadians(target/ ticks_in_degrees)) * f;
        double power = pid + ff;
        double ff1 = Math.cos(Math.toRadians(target/ ticks_in_degrees)) * f;
        double power1 = pid + ff1;
        sliderMotor.setPower(power);
        sliderMotorMotor.setPower(power1);
        // telemetry.addData("pos", slidePos);
        // telemetry.addData("target", target);
        // telemetry.update();
    }

    public void init(HardwareMap hwMap) {
        sliderMotor = hwMap.get(DcMotor.class, "slideMotor");
        sliderMotorMotor = hwMap.get(DcMotor.class, "slideMotorMotor");
        controller = new PIDController(p, i, d);
    }


    public void Sliderset1() {
        //up
        sliderMotor.setPower(0.6);
        sliderMotorMotor.setPower(0.6);
    }


    public void Sliderset2() {
        //down
        sliderMotor.setPower(-0.6);
        sliderMotorMotor.setPower(-0.6);
    }







}
