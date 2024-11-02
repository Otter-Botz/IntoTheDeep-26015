package org.firstinspires.ftc.teamcode.Autonomous.PIDAutonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.ArmSlider;
import org.firstinspires.ftc.teamcode.common.PID_Arm;
import org.firstinspires.ftc.teamcode.common.PID_Slider;
import org.firstinspires.ftc.teamcode.common.claw;
import org.firstinspires.ftc.teamcode.common.wrist;

public class BiddleSpecmenLM1 extends LinearOpMode {

    org.firstinspires.ftc.teamcode.common.claw claw = new claw();
    org.firstinspires.ftc.teamcode.common.ArmSlider ArmSlider = new ArmSlider();
    org.firstinspires.ftc.teamcode.common.PID_Arm PID_Arm = new PID_Arm();
    org.firstinspires.ftc.teamcode.common.PID_Slider PID_Slider = new PID_Slider();
    org.firstinspires.ftc.teamcode.common.wrist wrist = new wrist();
    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.2;// Max Turn speed to limit turn rate

    Servo leftPlate, rightPlate;
    @Override
    public void runOpMode() throws InterruptedException {
        CommonAuto commonAuto = new CommonAuto(this);
        claw.init(hardwareMap);
        ArmSlider.init(hardwareMap);
        PID_Arm.init(hardwareMap);
        PID_Slider.init(hardwareMap);
        wrist.init(hardwareMap);
        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");

        waitForStart();

        commonAuto.driveStraight(DRIVE_SPEED, 20);
        //Arm Code
        claw.set1();
        //Claw Open
        claw.set2();
        //Claw Close
        commonAuto.moveBack(DRIVE_SPEED,-20);




    }}