package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.ArmSlider;
import org.firstinspires.ftc.teamcode.common.PID_Arm;
import org.firstinspires.ftc.teamcode.common.PID_Slider;
import org.firstinspires.ftc.teamcode.common.claw;
import org.firstinspires.ftc.teamcode.common.wrist;

@Disabled
public class Biddle extends LinearOpMode {
    claw claw = new claw();
    ArmSlider ArmSlider = new ArmSlider();
    PID_Arm PID_Arm = new PID_Arm();
    PID_Slider PID_Slider = new PID_Slider();
    wrist wrist = new wrist();

    @Override
    public void runOpMode() throws InterruptedException {

        claw.init(hardwareMap);
        ArmSlider.init(hardwareMap);
        PID_Arm.init(hardwareMap);
        PID_Slider.init(hardwareMap);
        wrist.init(hardwareMap);
        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");

        /*
        waitForStart();

        while (opModeIsActive()) {
            new TrajectoryBuilder(new Pose2d())
                    .forward(40)
                    .straferight(30)
                    .build();
            //Move to rung
            PID_Slider.sliderMotor.setPower(0.5);
            PID_Slider.sliderMotorMotor.setPower(-0.5);
            PID_Slider.sliderMotorMotor.setTargetPosition(900);
            PID_Slider.sliderMotor.setTargetPosition(900);
            PID_Arm.armMotor.setPower(0.4);
            PID_Arm.armMotor.setTargetPosition(300);
            //Arm Motor to be tuned
            claw.set1();
            //Claw Open
            claw.set2();
            //Claw Close
            PID_Slider.sliderMotor.setPower(-0.5);
            PID_Slider.sliderMotorMotor.setPower(0.5);
            PID_Slider.sliderMotorMotor.setTargetPosition(-900);
            PID_Slider.sliderMotor.setTargetPosition(-900);
            PID_Arm.armMotor.setPower(-0.4);
            PID_Arm.armMotor.setTargetPosition(-300);
            //Arm Motor to be tuned
            // Robot moves to the specified coordinates while linearly
            // interpolating between the start heading and a specified end heading.
            new TrajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(40, 40, Math.toRadians(90)))
                    .build();
            //Slider Up/Claw outtake
            PID_Slider.sliderMotor.setPower(0.5);
            PID_Slider.sliderMotorMotor.setPower(-0.5);
            PID_Slider.sliderMotorMotor.setTargetPosition(900);
            PID_Slider.sliderMotor.setTargetPosition(900);
            PID_Arm.armMotor.setPower(0.4);
            PID_Arm.armMotor.setTargetPosition(300);
            //Slider Up
            claw.set1();
            //Claw Open
            claw.set2();
            //Claw Closed
            new TrajectoryBuilder(new Pose2d())
                    .back(23)
                    .strafeleft(17)
                    .forward(8);
            //Claw pick up
            //Wrist Positions
            //To be tuned
            claw.set1();
            //Claw Open
            claw.set2();
            //Claw Close
            new TrajectoryBuilder(new Pose2d())
                    .back(23)
                    .straferight(14)
                    .forward(13.5);
            //Slider up/Claw outtake
            PID_Slider.sliderMotor.setPower(0.5);
            PID_Slider.sliderMotorMotor.setPower(-0.5);
            PID_Slider.sliderMotorMotor.setTargetPosition(900);
            PID_Slider.sliderMotor.setTargetPosition(900);
            PID_Arm.armMotor.setPower(0.4);
            PID_Arm.armMotor.setTargetPosition(300);
            claw.set1();
            //Claw Open
            claw.set2();
            //Claw Close
            PID_Slider.sliderMotor.setPower(-0.5);
            PID_Slider.sliderMotorMotor.setPower(0.5);
            PID_Slider.sliderMotorMotor.setTargetPosition(-900);
            PID_Slider.sliderMotor.setTargetPosition(-900);
            PID_Arm.armMotor.setPower(-0.4);
            PID_Arm.armMotor.setTargetPosition(-300);

            new TrajectoryBuilder(new Pose2d())
                    .back(13)
                    .strafeleft(13.5)
                    .forward(8);
            //Robo to second spike
            claw.set1();
            //Claw Open
            claw.set2();
            // Close
            new TrajectoryBuilder(new Pose2d())
                    .back(30)
                    .straferight(20)
                    .forward(35);
            //Basket number 2
            PID_Slider.sliderMotor.setPower(0.5);
            PID_Slider.sliderMotorMotor.setPower(-0.5);
            PID_Slider.sliderMotorMotor.setTargetPosition(900);
            PID_Slider.sliderMotor.setTargetPosition(900);
            PID_Arm.armMotor.setPower(0.4);
            PID_Arm.armMotor.setTargetPosition(300);
            claw.set1();
            //Claw Open
            claw.set2();
            //Claw Close
            PID_Slider.sliderMotor.setPower(-0.5);
            PID_Slider.sliderMotorMotor.setPower(0.5);
            PID_Slider.sliderMotorMotor.setTargetPosition(-900);
            PID_Slider.sliderMotor.setTargetPosition(-900);
            PID_Arm.armMotor.setPower(-0.4);
            PID_Arm.armMotor.setTargetPosition(-300);

            new TrajectoryBuilder(new Pose2d())
                    .back(20)
                    .strafeleft(25)
                    .forward(35);
            //Robo to third spike
            claw.set1();
            //Claw Open
            claw.set2();
            //Claw close
            new TrajectoryBuilder(new Pose2d())
                    .back(20)
                    .strafeleft(25)
                    .forward(35);
            //Robo to Basket
            PID_Slider.sliderMotor.setPower(0.5);
            PID_Slider.sliderMotorMotor.setPower(-0.5);
            PID_Slider.sliderMotorMotor.setTargetPosition(900);
            PID_Slider.sliderMotor.setTargetPosition(900);
            PID_Arm.armMotor.setPower(0.4);
            PID_Arm.armMotor.setTargetPosition(300);
            //Arm Slider Code might need to be tuned
            claw.set1();
            //Claw Open
            claw.set2();
            //Claw Close
            PID_Slider.sliderMotor.setPower(-0.5);
            PID_Slider.sliderMotorMotor.setPower(0.5);
            PID_Slider.sliderMotorMotor.setTargetPosition(-900);
            PID_Slider.sliderMotor.setTargetPosition(-900);
            PID_Arm.armMotor.setPower(-0.4);
            PID_Arm.armMotor.setTargetPosition(-300);

            //slider
            //Slider up and outtake
            //Park?

            */
        }
    }



