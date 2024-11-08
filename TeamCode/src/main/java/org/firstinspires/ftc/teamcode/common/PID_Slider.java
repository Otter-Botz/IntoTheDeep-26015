package org.firstinspires.ftc.teamcode.common;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PID_Slider {

    public DcMotor sliderMotorMotor;
    public DcMotor sliderMotor;


    public void init(HardwareMap hwMap) {
        sliderMotor = hwMap.get(DcMotor.class, "slideMotor");
        sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        sliderMotorMotor = hwMap.get(DcMotor.class, "slideMotorMotor");
    }

    public void highbasket() {
        sliderMotor.setTargetPosition(900);
        sliderMotorMotor.setTargetPosition(-900);
    }

    public void lowbasket() {
        sliderMotor.setTargetPosition(500);
        sliderMotorMotor.setTargetPosition(-500);
    }

    public void restpos(){
        sliderMotor.setTargetPosition(0);
        sliderMotorMotor.setTargetPosition(0);
    }










}
