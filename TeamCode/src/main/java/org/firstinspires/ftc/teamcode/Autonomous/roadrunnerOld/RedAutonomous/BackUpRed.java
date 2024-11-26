package org.firstinspires.ftc.teamcode.Autonomous.roadrunnerOld.RedAutonomous;
/*
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Common.AutoMainSliders;
//import org.firstinspires.ftc.teamcode.Autonomous.Common.armSlide;
//import org.firstinspires.ftc.teamcode.Autonomous.Common.armSlide;
import org.firstinspires.ftc.teamcode.Autonomous.Common.autoClaw;
import org.firstinspires.ftc.teamcode.Autonomous.Common.autoWrist;
import org.firstinspires.ftc.teamcode.Roadrunnerlol.MecanumDrive;

@Autonomous
public class BackUpRed extends LinearOpMode {




    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(16, -62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        org.firstinspires.ftc.teamcode.Autonomous.Common.autoClaw autoClaw = new autoClaw(hardwareMap);
        //armSlide armMotor = new armSlide(hardwareMap);
        autoWrist wristServo = new autoWrist(hardwareMap);
        //armSlide slideServo = new armSlide(hardwareMap);
        autoClaw clawServo = new autoClaw(hardwareMap);
        AutoMainSliders AutoMainSliders = new AutoMainSliders(hardwareMap);


        waitForStart();

        double Tab1X = 0;
        double Tab1Y = -38;
        double Tab2X = 60;
        double Tab2Y = -55;
        double Tab3X = 72;
        double Tab3Y = -55;
        double Tab4X = 0;
        double Tab4Y = -38;
        double Tab5X = 52;
        double Tab5Y = -57;
        double Tab6X = 56;
        double Tab6Y = -20;
        double Tab7X = 0;
        double Tab7Y = -38;


        TrajectoryActionBuilder ScorePreloadSpecimen = drive.actionBuilder(initialPose)
                //Score Preloaded Specimen
                .strafeToLinearHeading(new Vector2d(0, -38), Math.toRadians(270));
        TrajectoryActionBuilder MoveandPushFirstSample = drive.actionBuilder(new Pose2d(Tab1X, Tab1Y, Math.toRadians(270)))
                //Sample One
                .strafeToLinearHeading(new Vector2d(48, -38), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(44, -5), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(60, -5), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(60, -55), Math.toRadians(270))
                .waitSeconds(0);
        TrajectoryActionBuilder MoveandPushSecondSample = drive.actionBuilder(new Pose2d(Tab2X, Tab2Y, Math.toRadians(270)))
                //Second Sample
                .strafeToLinearHeading(new Vector2d(60, -5), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(72, -5), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(72, -55), Math.toRadians(270))
                .waitSeconds(0);
        TrajectoryActionBuilder ScoreSpecimen1 = drive.actionBuilder(new Pose2d(Tab3X, Tab3Y, Math.toRadians(270)))
                //Score 1
                .strafeToLinearHeading(new Vector2d(57, -55), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(0, -38), Math.toRadians(270))
                .waitSeconds(0);
        TrajectoryActionBuilder ScoreSpecimen2 = drive.actionBuilder(new Pose2d(Tab4X, Tab4Y, Math.toRadians(270)))
                //Score 2
                .strafeToLinearHeading(new Vector2d(57, -55), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(0, -38), Math.toRadians(270))
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
        Action trajectoryActionCloseOut = ScoreSpecimen2.fresh()
                .strafeTo(new Vector2d(59, -50))
                .build();


        Actions.runBlocking(
                new SequentialAction(
                        ScorePreloadSpecimen.build(),
                        AutoMainSliders.HighRung(),
                        MoveandPushFirstSample.build(),
                        MoveandPushSecondSample.build(),
                        ScoreSpecimen1.build(),
                        AutoMainSliders.HighRung(),
                        ScoreSpecimen2.build(),
                        AutoMainSliders.HighRung(),
                        trajectoryActionCloseOut
                )
        );


    }
}
*/