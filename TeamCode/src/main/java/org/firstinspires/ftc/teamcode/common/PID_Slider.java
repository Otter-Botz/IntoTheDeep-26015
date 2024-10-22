package org.firstinspires.ftc.teamcode.common;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PID_Slider {

    DcMotor sliderMotorMotor;
    DcMotor sliderMotor;


    public void init(HardwareMap hwMap) {
        sliderMotor = hwMap.get(DcMotor.class, "slideMotor");
        sliderMotorMotor = hwMap.get(DcMotor.class, "slideMotorMotor");
    }









}
