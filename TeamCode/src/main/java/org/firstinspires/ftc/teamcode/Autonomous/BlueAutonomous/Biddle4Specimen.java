package org.firstinspires.ftc.teamcode.Autonomous.BlueAutonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//  ryan says "code at bad is aryan  ;) "
import org.firstinspires.ftc.teamcode.Autonomous.Common.autoWrist;
import org.firstinspires.ftc.teamcode.Autonomous.Common.PID_Arm;
import org.firstinspires.ftc.teamcode.Autonomous.Common.armSlide;
import org.firstinspires.ftc.teamcode.Autonomous.Common.autoClaw;
import org.firstinspires.ftc.teamcode.Roadrunnerlol.MecanumDrive;





@Autonomous
public class Biddle4Specimen extends LinearOpMode {


    ElapsedTime time = new ElapsedTime();
    double armtime = time.seconds();

    public void armWaitTime() {
        if (time.seconds() >= 1) {
            stop();
        }
    }
    //chucky


    @Override
    public void runOpMode() {
        PID_Arm armMotor = new PID_Arm(hardwareMap);
        autoWrist wristServo = new autoWrist(hardwareMap);
        armSlide slideServo = new armSlide(hardwareMap);
        autoClaw clawServo = new autoClaw(hardwareMap);

        Pose2d initialPose = new Pose2d(-16, 62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);



         double lastX = -57;
         double lastY = 40;
         double nextX = -69;
         double nextY = 40;
         boolean active;
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

        while (!isStopRequested() && !opModeIsActive()) {

        }


        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(

                        clawServo.clawClose(),
                       // armMotor.armUp(),
                        wristServo.wristUp(),
                        score1Transfer1.build(),
                        /*
                        clawServo.clawOpen(),
                        armMotor.armDown(),
                        clawServo.clawClose(),
                         armMotor.backDown(),
                        clawServo.clawOpen(),
                        sampleTransfer2.build(),
                        armMotor.armDown(),
                        clawServo.clawClose(),
                        armMotor.backDown(),
                        clawServo.clawOpen(),
                        sampleTransfer3.build(),
                         armMotor.armDown(),
                        clawServo.clawClose(),
                         armMotor.backDown(),
                         clawServo.clawOpen(),
                        specimenPickup1.build(),
                        armMotor.backDown(),
                        clawServo.clawClose(),
                        armMotor.armUp(),
                        score1Pickup2.build(),
                        clawServo.clawOpen(),
                         armMotor.backDown(),
                        clawServo.clawClose(),
                         armMotor.armUp(),
                        score2Pickup3.build(),
                        clawServo.clawOpen(),
                        armMotor.backDown(),
                       clawServo.clawClose(),
                       armMotor.armUp(),
                        score3.build(),

                         */


                        parkCloseOut




                )
        );
    }
}