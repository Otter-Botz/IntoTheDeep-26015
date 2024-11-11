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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.Autonomous.Common.autoTouch;
import org.firstinspires.ftc.teamcode.Autonomous.Common.autoWrist;
import org.firstinspires.ftc.teamcode.Autonomous.Common.autoArmSlider;
import org.firstinspires.ftc.teamcode.Autonomous.Common.armSlide;
import org.firstinspires.ftc.teamcode.Autonomous.Common.autoArmSlider;
import org.firstinspires.ftc.teamcode.Autonomous.Common.autoClaw;
import org.firstinspires.ftc.teamcode.Roadrunnerlol.MecanumDrive;

@Autonomous
public class Biddle4Specimen extends LinearOpMode {



    @Override
    public void runOpMode() {
        autoArmSlider armMotor = new autoArmSlider(hardwareMap);
        autoWrist wristServo = new autoWrist(hardwareMap);
        armSlide slideServo = new armSlide(hardwareMap);
        autoClaw clawServo = new autoClaw(hardwareMap);
        autoTouch touchreset = new autoTouch(hardwareMap);
        Pose2d initialPose = new Pose2d(-16, 62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


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

                        clawServo.clawClose(),
                        armMotor.mathRun(),
                        armMotor.armUp(),
                        wristServo.wristUp(),
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
