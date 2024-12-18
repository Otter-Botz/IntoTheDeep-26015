package org.firstinspires.ftc.teamcode.common;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.interfaces.armSystem;



public class ArmSlider  {
     static Servo armSliderServo;
     public double out = 1;
     public double in = 0;
     public double middle = 0.5;
    public static void init(HardwareMap hwMap) {
        armSliderServo = hwMap.get(Servo.class, "servoSlide");

    }


    public void set(double pos) {
        armSliderServo.setPosition(pos);
    }


}

