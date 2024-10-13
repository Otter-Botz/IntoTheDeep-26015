package org.firstinspires.ftc.teamcode.common;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TeleCommon  {
    private HardwareMap hardwareMap;
    DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
    DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
    DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");
    DcMotor leftBack = hardwareMap.dcMotor.get("leftBack");
    Servo servo1 = hardwareMap.servo.get("servo1");
    CRServo servo2 = hardwareMap.crservo.get("servo2");
    DcMotor slideMotor1 = hardwareMap.dcMotor.get("slideMotor1");
    DcMotor slideMotor2 = hardwareMap.dcMotor.get("slideMotor2");
    DcMotor armMotor = hardwareMap.dcMotor.get("armMotor");

    public void clawOpen() {
        servo1.setPosition(0.8);
    }
    public void clawClose() {
        servo1.setPosition(0.5);
    }

    public void slideUp(){

    }
    public void slideDown() {

    }





}
