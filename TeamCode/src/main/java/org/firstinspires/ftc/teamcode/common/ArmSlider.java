package org.firstinspires.ftc.teamcode.common;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.interfaces.armSystem;



public class ArmSlider  {
     static CRServo armSliderServo;

    public static void init(HardwareMap hwMap) {
        armSliderServo = hwMap.get(CRServo.class, "servoSlide");

    }

    public void armslideGoToPos(){
        armSliderServo.setPower(0.5);


    }

    public void setMode(DcMotor.RunMode runMode) {
    }

    public void setTargetPosition(int i) {
    }
}

