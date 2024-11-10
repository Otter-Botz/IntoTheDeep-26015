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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Roadrunnerlol.MecanumDrive;

@Autonomous
public class Biddle4Specimen extends LinearOpMode {

    public static double p = 0.005, i = 0.03, d = 0.0005;
    public static double f = 0.12;

    public double target = -750;

    private final double ticks_in_degrees = 700 / 180;

    //auto claw stuff
    public class autoClaw {
        public Servo clawServo;

        //init stuff
        public autoClaw(HardwareMap hardwareMap) {
            clawServo = hardwareMap.get(Servo.class, "clawServo");
        }

        //claw open
        public class open implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                target = 0.4;
                return false;
            }
        }

        public Action clawOpen() {
            return new open();
        }

        //claw close
        public class close implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                target = 0.7;
                return false;
            }
        }

        public Action clawClose() {
            return new close();
        }
    }


    //auto wrist stuff
    public class autoWrist {
        public Servo wristServo;


        public autoWrist(HardwareMap hardwareMap) {
            wristServo = hardwareMap.get(Servo.class, "wristServo");
        }
        //wrist up
        public class wristup implements Action{

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                target = 0.4;
                return false;
            }
        }

        public class wristdown implements Action{

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                target = 0.1;
                return false;
            }
        }
    }


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
    public class backUp implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            target = -100;
            return false;
        }
    }

    public class backDown implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            target = -200;
            return false;
        }

    }
    public Action backDown(){
        return new backDown();
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

    @Override
    public void runOpMode() {
        autoArm armMotor = new autoArm(hardwareMap);
        Pose2d initialPose = new Pose2d(-16, 62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
       // armMotor.math();

         double lastX = -57;
         double lastY = 40;
         double nextX = -69;
         double nextY = 40;
        TrajectoryActionBuilder score1Transfer1 = drive.actionBuilder(initialPose)

                .strafeToConstantHeading(new Vector2d(0, 33))
                .strafeToConstantHeading(new Vector2d(lastX, lastY))
                .waitSeconds(1);

        TrajectoryActionBuilder sampleTransfer2 = drive.actionBuilder(
                        new Pose2d(lastX, lastY, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(nextX, nextY))
                .waitSeconds(1);

        lastX = nextX;
        lastY = nextY;

        TrajectoryActionBuilder sampleTransfer3 = drive.actionBuilder (
                new Pose2d(lastX, lastY, Math.toRadians(270)))
                .turnTo(268)
                .waitSeconds(1);

        nextX=-57;
        nextY=55;
        TrajectoryActionBuilder specimenPickup1 = drive.actionBuilder(
                new Pose2d(lastX, lastY, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(nextX,nextY), Math.toRadians(270))
                .waitSeconds(1);

        lastX = nextX;
        lastY = nextY;

        nextX = -57;
        nextY = 55;
        TrajectoryActionBuilder score1Pickup2 = drive.actionBuilder( new Pose2d(lastX, lastY, Math.toRadians(270)))

                .strafeToLinearHeading(new Vector2d(0, 42), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(0, 30), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(nextX, nextY), Math.toRadians(270))
                .waitSeconds(1);

        lastX = nextX;
        lastY = nextY;
        TrajectoryActionBuilder score2Pickup3 = drive.actionBuilder( new Pose2d(lastX, lastY, Math.toRadians(270)))

                .strafeToLinearHeading(new Vector2d(0, 42), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(0, 30), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(nextX, nextY), Math.toRadians(270))
                .waitSeconds(1);

        TrajectoryActionBuilder score3 = drive.actionBuilder( new Pose2d(lastX, lastY, Math.toRadians(270)))

                .strafeToLinearHeading(new Vector2d(0, 42), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(0, 30), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(nextX, nextY), Math.toRadians(270));


        Action parkCloseOut = score1Transfer1.fresh()
                .strafeToLinearHeading(new Vector2d(-59, 60), Math.toRadians(270))
                .build();

        // actions that need to happen on init; for instance, a claw tightening.


        while (!isStopRequested() && !opModeIsActive()) {


        }


        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                       // armMotor.armUp(),
                        score1Transfer1.build(),
                       // autoClaw.open(),
                      //  armMotor.armDown(),
                       // autoWrist.wristdown(),
                      //  autoClaw.close(),
                        // armMotor.backDown(),
                        //autoClaw.open(),
                        sampleTransfer2.build(),
                       // armMotor.armDown(),
                      // autoWrist.wristdown(),
                       // autoClaw.close(),
                       // armMotor.backDown(),
                       // autoClaw.open(),
                        sampleTransfer3.build(),
                        // armMotor.armDown(),
                        // autoWrist.wristdown()
                        // autoClaw.close(),
                        // armMotor.backDown(),
                        // autoClaw.open(),
                        specimenPickup1.build(),
                        //armMotor.armUp(),
                       //autoClaw.open
                       // armMotor.backUp(),
                        // autoClaw.close(),
                        score1Pickup2.build(),
                        // armMotor.armUp(),
                       //autoClaw.open(),
                        // armMotor.backUp(),
                        // autoClaw.close(),
                       // armMotor.armUp(),
                        // autoClaw.open(),
                        score2Pickup3.build(),
                        //autoClaw.open(),
                        //armMotor.backUp(),
                        //autoClaw.close
                        score3.build(),
                        // armMotor.armUp(),
                        // autoClaw.open(),
                        // armMotor.armDown(),
                        parkCloseOut


                )
        );
    }
}
