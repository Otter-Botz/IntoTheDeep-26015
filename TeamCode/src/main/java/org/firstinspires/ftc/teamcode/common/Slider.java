package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slider {

    public DcMotor sliderMotorMotor;
    public DcMotor sliderMotor;


    public void init(HardwareMap hwMap) {
        sliderMotor = hwMap.get(DcMotor.class, "slideMotor");
        sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        sliderMotorMotor = hwMap.get(DcMotor.class, "slideMotorMotor");
    }














}
