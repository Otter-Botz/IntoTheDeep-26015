package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.interfaces.armSystem;

public class wrist implements armSystem {
    Servo wristServo;

    @Override
    public void set1() {
        wristServo.setPosition(0.4);
    }

    @Override
    public void set2() {
        wristServo.setPosition(0.6);
    }

    @Override
    public void init(HardwareMap hwMap) {
        wristServo = hwMap.get(Servo.class,  "wristServo");
    }
}
