package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.ArmSlider;
import org.firstinspires.ftc.teamcode.common.PID_Arm;
import org.firstinspires.ftc.teamcode.common.PID_Slider;
import org.firstinspires.ftc.teamcode.common.claw;
import org.firstinspires.ftc.teamcode.common.wrist;

@Autonomous(name = "Bleft")
public class Bleft extends LinearOpMode {
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


        waitForStart();

        while (opModeIsActive()) {
            new TrajectoryBuilder(new Pose2d())
                    .forward(40)
                    .straferight(30)
                    .build();
            //Move to rung
            // Robot moves to the specified coordinates while linearly
            // interpolating between the start heading and a specified end heading.
            new TrajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(40, 40, Math.toRadians(90)))
                    .build();
            //Slider Up/Claw outtake
            new TrajectoryBuilder(new Pose2d())
                    .back(23)
                    .strafeleft(17)
                    .forward(8);
            //Claw pick up
            new TrajectoryBuilder(new Pose2d())
                    .back(23)
                    .straferight(14)
                    .forward(13.5);
            //Slider up/Claw outtake
            new TrajectoryBuilder(new Pose2d())
                    .back(13)
                    .strafeleft(13.5)
                    .forward(8);
            //Robo to second spike
            new TrajectoryBuilder(new Pose2d())
                    .back(30)
                    .straferight(20)
                    .forward(35);
            //Basket number 2
            new TrajectoryBuilder(new Pose2d())
                    .back(20)
                    .strafeleft(25)
                    .forward(35);
            //Robo to third spike
            new TrajectoryBuilder(new Pose2d())
                    .back(20)
                    .strafeleft(25)
                    .forward(35);
            //Robo to Basket
            //slidermotor.setpower(0.5);
            //slidermotormotor.setpower(0.5);
            //slidermotor.setTargetPosition(1350);

            //slider
            //Slider up and outtake
            //Park?











        }
    }
}


