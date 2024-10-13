package org.firstinspires.ftc.teamcode.common;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.interfaces.armSlider;



public class ArmSlider implements armSlider {
    private CRServo armSliderServo;

    @Override
    public void setPowerMove() {
        armSliderServo.setPower(0.5);

    }

    @Override
    public void init(HardwareMap hwMap) {
        armSliderServo = hwMap.get(CRServo.class, "armSliderServo");
    }
}

