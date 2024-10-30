package org.firstinspires.ftc.teamcode.common;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.interfaces.armSystem;


public class claw implements armSystem {

    public Servo clawServo;

    @Override
    public void set1() {
        //open
        clawServo.setPosition(0.4);

    }

    @Override
    public void set2() {
        //close
        clawServo.setPosition(0.7);
    }

    @Override
    public void init(HardwareMap hwMap) {
        clawServo = hwMap.get(Servo.class,  "clawServo");

    }


}
