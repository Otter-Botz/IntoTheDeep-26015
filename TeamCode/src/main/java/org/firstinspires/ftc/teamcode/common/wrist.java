package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.interfaces.armSystem;

public class wrist implements armSystem {
    public Servo wristServo;
    //submersible
    public double up = 0.5;

    public double AutoUp = 0.25;
    public double AutoDown = 0;
    public double AutoMiddle = 0.25;
    // high basket
    public double down = 0;




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

    public void up (){
        wristServo.setPosition(0.5);
    }
    public void autoUp (){
        wristServo.setPosition(0.25);
    }
    public void down (){
        wristServo.setPosition(0);
    }
}