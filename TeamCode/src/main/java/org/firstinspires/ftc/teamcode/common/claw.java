package org.firstinspires.ftc.teamcode.common;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.interfaces.armSystem;


public class claw implements armSystem {

    private Servo clawServo;





    @Override
    public void set1() {
        //open
        clawServo.setPosition(0.5);

    }

    @Override
    public void set2() {
        //down
        clawServo.setPosition(0.8);
    }

    @Override
    public void init(HardwareMap hwMap) {
        clawServo = hwMap.get(Servo.class,  "clawServo");

    }
}
