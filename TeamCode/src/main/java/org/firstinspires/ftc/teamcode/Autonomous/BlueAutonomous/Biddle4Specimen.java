package org.firstinspires.ftc.teamcode.Autonomous.BlueAutonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Roadrunnerlol.MecanumDrive;


public class Biddle4Specimen extends LinearOpMode {

    public static double p = 0.005, i = 0.03, d = 0.0005;
    public static double f = 0.12;

    public double target = -750;

    private final double ticks_in_degrees = 700 / 180;

// auto arm stuff
    public class autoArm {
        private PIDController controller;
        public DcMotor armMotor;

        //init
        public autoArm(HardwareMap hardwareMap) {
            armMotor = hardwareMap.get(DcMotor.class, "armMotor");
            controller = new PIDController(p, i, d);
        }

        //arm  up stuff
        public class up implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                target = -65;
                return false;
            }

        }
        public Action armUp(){
            return new up();
        }

        // arm all the way down
        public class down implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                target = -700;
                return false;
            }

        }
        public Action armDown(){
            return new down();
        }

        //place specimen on rung for arm
        public class rung implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                target = -300;
                return false;
            }

        }
        public Action armDownOnRung(){
            return new rung();
        }

        //math for PID
        public void math() {


            controller.setPID(p, i , d);
            int slidePos = armMotor.getCurrentPosition();
            double pid = controller.calculate(slidePos, target);
            double ff = Math.cos(Math.toRadians(target/ ticks_in_degrees)) * f;
            double power = pid + ff;
            armMotor.setPower(power);
        }
    }


    public class claw {}




    @Override
    public void runOpMode() throws InterruptedException {
        autoArm armMotor = new autoArm(hardwareMap);
        Pose2d initialPose = new Pose2d(-16, 70, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        armMotor.math();


        TrajectoryActionBuilder specimenPreload = drive.actionBuilder(initialPose)

                .strafeToLinearHeading(new Vector2d(0, 36), Math.toRadians(270))
                .waitSeconds(1);
        TrajectoryActionBuilder sampleTransfer1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-47, 42), Math.toRadians(270))
                .waitSeconds(1);
        TrajectoryActionBuilder sampleTransfer2 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-59, 42), Math.toRadians(270))
                .waitSeconds(1);
        TrajectoryActionBuilder sampleTransfer3 = drive.actionBuilder(initialPose)
                .turn(-0.5)
                .waitSeconds(1);
        TrajectoryActionBuilder specimenPickup1 = drive.actionBuilder(initialPose)
                .turn(0.5)
                .strafeToLinearHeading(new Vector2d(-47, 50), Math.toRadians(270))
                .waitSeconds(1);
        TrajectoryActionBuilder specimenScore1 = drive.actionBuilder(initialPose)

                .strafeToLinearHeading(new Vector2d(0, 36), Math.toRadians(270))
                .waitSeconds(1);
        TrajectoryActionBuilder specimenPickup2 = drive.actionBuilder(initialPose)

                .strafeToLinearHeading(new Vector2d(-47, 50), Math.toRadians(270))
                .waitSeconds(1);
        TrajectoryActionBuilder specimenScore2 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(0, 36), Math.toRadians(270))
                .waitSeconds(1);
        TrajectoryActionBuilder specimenPickup3 = drive.actionBuilder(initialPose)

                .strafeToLinearHeading(new Vector2d(-47, 50), Math.toRadians(270))
                .waitSeconds(1);
        TrajectoryActionBuilder scoreAndPark = drive.actionBuilder(initialPose)

                .strafeToLinearHeading(new Vector2d(0, 36), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-59, 42), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-59, 60), Math.toRadians(270));




        Action trajectoryActionCloseOut = specimenPreload.fresh()
                .build();

        // actions that need to happen on init; for instance, a claw tightening.


        while (!isStopRequested() && !opModeIsActive()) {


        }


        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        specimenPreload.build(),
                        armMotor.armUp(),
                        armMotor.armDownOnRung(),
                        sampleTransfer1.build(),
                        armMotor.armDown(),
                        trajectoryActionCloseOut
                )
        );
    }
}
