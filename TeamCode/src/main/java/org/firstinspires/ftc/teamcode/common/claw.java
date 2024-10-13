package org.firstinspires.ftc.teamcode.common;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.interfaces.clawMan;


public class claw implements clawMan {

    private Servo clawServo;
    @Override
    public void close()  {
        clawServo.setPosition(0.8);
    }

    @Override
    public void open() {
        clawServo.setPosition(0.4);
    }

    @Override
    public void init(HardwareMap hwMap) {
        clawServo = hwMap.get(Servo.class,  "clawServo");

    }
}
