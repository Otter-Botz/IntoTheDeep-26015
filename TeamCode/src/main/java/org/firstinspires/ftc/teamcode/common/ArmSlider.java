package org.firstinspires.ftc.teamcode.common;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.interfaces.armSystem;



public class ArmSlider implements armSystem {
    private CRServo armSliderServo;





    @Override
    public void set1() {
        //up
        armSliderServo.setPower(0.5);
    }

    @Override
    public void set2() {
        //down
        armSliderServo.setPower(-0.5);
    }

    @Override
    public void init(HardwareMap hwMap) {
        armSliderServo = hwMap.get(CRServo.class, "armSliderServo");

    }
}

