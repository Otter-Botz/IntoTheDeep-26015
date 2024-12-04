package org.firstinspires.ftc.teamcode.common;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.interfaces.armSystem;


public class claw implements armSystem {
    public Servo clawServo;
    public double open = 0.25;
    public double close = 0.01 ;


   // double clawServoPos = clawServo.getPosition();
   // int ArmPos = PID_Arm.armMotor.getCurrentPosition() + 20;
    /*
    if (clawServoPos == 0.3){
    PID_Arm.armMotor.setTargetPosition(ArmPos);
        }

     */
    public void AutoOpen() {
        clawServo.setPosition(0.55);
    }

    public void AutoClose() {
        clawServo.setPosition(0.37);
    }
    @Override
    public void set(double position) {
        //open
        clawServo.setPosition(position);
    }

    @Override
    public double getPosition(){
        return clawServo.getPosition();
    }


    @Override
    public void init(HardwareMap hwMap) {
        clawServo = hwMap.get(Servo.class,  "clawServo");
    }








}
