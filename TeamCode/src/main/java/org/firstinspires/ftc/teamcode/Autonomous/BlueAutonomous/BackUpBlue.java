package org.firstinspires.ftc.teamcode.Autonomous.BlueAutonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Autonomous.Common.AutoMainSliders;
//import org.firstinspires.ftc.teamcode.Autonomous.Common.armSlide;
//import org.firstinspires.ftc.teamcode.Autonomous.Common.armSlide;
import org.firstinspires.ftc.teamcode.Autonomous.Common.PID_Arm;
import org.firstinspires.ftc.teamcode.Autonomous.Common.autoClaw;
import org.firstinspires.ftc.teamcode.Autonomous.Common.autoWrist;
import org.firstinspires.ftc.teamcode.Roadrunnerlol.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.ArmSlider;
import org.firstinspires.ftc.teamcode.Autonomous.Common.PID_Arm;
import org.firstinspires.ftc.teamcode.common.Slider;
import org.firstinspires.ftc.teamcode.common.claw;
import org.firstinspires.ftc.teamcode.common.wrist;

@Autonomous(name = "1+2Push")
public class BackUpBlue extends LinearOpMode {

    private PIDController controller;
    public DcMotor armMotor;
    public TouchSensor touchSensor;
    public static double p = 0.005, i = 0, d = 0.0001;
    public static double f = 0.01;
    double ARM_UP = 600;
    double ARM_DOWN = 70;
    double ARM_BACK = 1700;
    double ARM_START = 180;
    public double target = ARM_START;

    private final double ticks_in_degrees = 2786.2 / 360;

    public class math implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            controller.setPID(p, i , d);
            int armPos = armMotor.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target/ ticks_in_degrees)) * f;
            double power = pid + ff;
            armMotor.setPower(power);
            return true;
        }
    }
//    public Action mathRun() {
//        return new PID_Arm.math();
//    }


    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(-16, 62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        org.firstinspires.ftc.teamcode.Autonomous.Common.autoClaw autoClaw = new autoClaw(hardwareMap);
       // armSlide armMotor = new armSlide(hardwareMap);
        autoWrist wristServo = new autoWrist(hardwareMap);
       // armSlide slideServo = new armSlide(hardwareMap);
        autoClaw clawServo = new autoClaw(hardwareMap);
        AutoMainSliders AutoMainSliders = new AutoMainSliders(hardwareMap);


        waitForStart();

        double originalTabX = -16;
        double originalTabY = 62;
        double Tab1X = 0;
        double Tab1Y = 38;
        double Tab2X = -60;
        double Tab2Y = 55;
        double Tab3X = -72;
        double Tab3Y = 55;
        double Tab4X = 0;
        double Tab4Y = 38;
        double Tab5X = 52;
        double Tab5Y = 57;
        double Tab6X = 56;
        double Tab6Y = 20;
        double Tab7X = 0;
        double Tab7Y = 38;


        TrajectoryActionBuilder StartingPosition = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-17, 60), Math.toRadians(270));
//        TrajectoryActionBuilder ScorePreloadSpecimen = drive.actionBuilder(initialPose)
//                //Score Preloaded Specimen
//                .strafeToLinearHeading(new Vector2d(0, 38), Math.toRadians(270));
//        TrajectoryActionBuilder MoveMore = drive.actionBuilder(new Pose2d(Tab4X, Tab4Y, Math.toRadians(270)));
        TrajectoryActionBuilder MoveandPushFirstSample = drive.actionBuilder(new Pose2d(originalTabX
                        , originalTabY, Math.toRadians(270)))
                //Sample One
                .strafeToLinearHeading(new Vector2d(-48, 38), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-44, 5), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-60, 5), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-60, 55), Math.toRadians(270))
                .waitSeconds(0);
        TrajectoryActionBuilder MoveandPushSecondSample = drive.actionBuilder(new Pose2d(Tab2X, Tab2Y, Math.toRadians(270)))
                //Second Sample
                .strafeToLinearHeading(new Vector2d(-60, 5), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-72, 5), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-72, 55), Math.toRadians(270))
                .waitSeconds(0);
//        TrajectoryActionBuilder ScoreSpecimen1 = drive.actionBuilder(new Pose2d(Tab3X, Tab3Y, Math.toRadians(270)))
//                //Score 1
//                .strafeToLinearHeading(new Vector2d(-57, 55), Math.toRadians(270))
//                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(0, 38), Math.toRadians(270))
//                .waitSeconds(0);
//        TrajectoryActionBuilder ScoreSpecimen2 = drive.actionBuilder(new Pose2d(Tab4X, Tab4Y, Math.toRadians(270)))
//                //Score 2
//                .strafeToLinearHeading(new Vector2d(-57, 55), Math.toRadians(270))
//                .waitSeconds(0)
//                .strafeToLinearHeading(new Vector2d(0, 38), Math.toRadians(270))
//                .waitSeconds(2);


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
        Action trajectoryActionCloseOut = MoveandPushSecondSample.fresh()
                .strafeTo(new Vector2d(-59, 55))
                .build();


        Actions.runBlocking(
                new SequentialAction(
                  StartingPosition.build(),
                        //ScorePreloadSpecimen.build(),
                        //MoveMore.build(),
//                        AutoMainSliders.HighRung(),
//                        AutoMainSliders.SliderIdlePosition(),
//                        AutoMainSliders.SliderDown(),
                        MoveandPushFirstSample.build(),
                        MoveandPushSecondSample.build(),
//                        //ScoreSpecimen1.build(),
                        //AutoMainSliders.HighRung(),
                        //AutoMainSliders.SliderIdlePosition(),
                        //AutoMainSliders.SliderDown(),
//                        ScoreSpecimen2.build(),
//                        AutoMainSliders.HighRung(),
//                        AutoMainSliders.SliderIdlePosition(),
//                        AutoMainSliders.SliderDown(),
                        trajectoryActionCloseOut
                )
        );


    }
}
