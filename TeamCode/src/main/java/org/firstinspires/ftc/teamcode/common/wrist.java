package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.interfaces.armSystem;

public class wrist implements armSystem {
    Servo wristServo;

    @Override
    //b
    //Open
    public void set1() {
        wristServo.setPosition(0.6);
    }

    @Override

    //a
    //CLose
    public void set2() {
        wristServo.setPosition(0.3);
    }

    @Override
    public void init(HardwareMap hwMap) {
        wristServo = hwMap.get(Servo.class,  "wristServo");
    }
}
