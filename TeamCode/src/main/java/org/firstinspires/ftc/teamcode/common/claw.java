package org.firstinspires.ftc.teamcode.common;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.interfaces.armSystem;


public class claw implements armSystem {

    private Servo clawServo;

    double servoPosition = clawServo.getPosition();




    @Override
    public void set1() {
        //open
        clawServo.setPosition(0.1);

    }

    @Override
    public void set2() {
        //close
        clawServo.setPosition(0.35);
    }

    @Override
    public void init(HardwareMap hwMap) {
        clawServo = hwMap.get(Servo.class,  "clawServo");

    }
}
