package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.interfaces.armSystem;

public class wrist implements armSystem {
    private Servo wristServo;
    @Override
    public void set1() {
        wristServo.setPosition(0.1);
    }

    @Override
    public void set2() {
        wristServo.setPosition(0.35);
    }

    @Override
    public void init(HardwareMap hwMap) {
        wristServo = hwMap.get(Servo.class,  "wristServo");
    }
}
