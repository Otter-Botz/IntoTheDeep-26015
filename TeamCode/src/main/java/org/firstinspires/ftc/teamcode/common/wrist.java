package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.interfaces.armSystem;

public class wrist implements armSystem {
    Servo wristServo;
    double up = 0.4;
    double down = 0.6;

    @Override
    public void set(double position) {
        wristServo.setPosition(position);
    }

    @Override
    public double getPosition() {
        return wristServo.getPosition();
    }

    @Override
    public void init(HardwareMap hwMap) {
        wristServo = hwMap.get(Servo.class,  "wristServo");
    }
}