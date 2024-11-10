package org.firstinspires.ftc.teamcode.Autonomous.BlueAutonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Roadrunnerlol.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.ArmSlider;
import org.firstinspires.ftc.teamcode.common.PID_Arm;
import org.firstinspires.ftc.teamcode.common.Slider;
import org.firstinspires.ftc.teamcode.common.claw;
import org.firstinspires.ftc.teamcode.common.wrist;

@Autonomous
public class BackUpBlue extends LinearOpMode {
    org.firstinspires.ftc.teamcode.common.claw claw = new claw();
    org.firstinspires.ftc.teamcode.common.ArmSlider ArmSlider = new ArmSlider();
    org.firstinspires.ftc.teamcode.common.PID_Arm PID_Arm = new PID_Arm();
    Slider PID_Slider = new Slider();
    org.firstinspires.ftc.teamcode.common.wrist wrist = new wrist();

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(-16, 62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        claw.init(hardwareMap);
        ArmSlider.init(hardwareMap);
        PID_Arm.init(hardwareMap);
        PID_Slider.init(hardwareMap);
        wrist.init(hardwareMap);
        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");

        waitForStart();

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                //Score Preloaded Specimen
                .strafeToLinearHeading(new Vector2d(0, 38), Math.toRadians(270))
                .waitSeconds(2)
                //Sample One
                .strafeToLinearHeading(new Vector2d(-48, 38), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-44, 5), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-60, 5), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-60, 55), Math.toRadians(270))
                .waitSeconds(2)
        //Second Sample
                .strafeToLinearHeading(new Vector2d(-60, 5), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-72, 5), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-72, 55), Math.toRadians(270))
                .waitSeconds(2)
//              Score 1
                .strafeToLinearHeading(new Vector2d(-57, 55), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(0, 38), Math.toRadians(270))
                .waitSeconds(2)
        //Score 2
                .strafeToLinearHeading(new Vector2d(-57, 55), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(0, 38), Math.toRadians(270))
                .waitSeconds(2);


//                .strafeToLinearHeading(new Vector2d(-55, 10), Math.toRadians(270))
//                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(-55, 62), Math.toRadians(270))
//                .waitSeconds(2)
//
//                //.strafeToLinearHeading(new Vector2d(-55, 10), Math.toRadians(270))
//                //.waitSeconds(2)
////                .strafeToLinearHeading(new Vector2d(-62, 10), Math.toRadians(270))
////                .waitSeconds(2)
////                .strafeToLinearHeading(new Vector2d(-62, 62), Math.toRadians(270))
////                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(0, 62), Math.toRadians(270))
//                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(0, 38), Math.toRadians(270))
//                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(0, 60), Math.toRadians(270))
//                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(-55, 60), Math.toRadians(270))
//                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(0, 60), Math.toRadians(270))
//                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(0, 38), Math.toRadians(270))
//                .waitSeconds(2);
        Action trajectoryActionCloseOut = tab1.fresh()
                .strafeTo(new Vector2d(-59, 50))
                .build();


        Actions.runBlocking(
                new SequentialAction(
                        tab1.build(),
                         trajectoryActionCloseOut
                )
        );


    }
}
