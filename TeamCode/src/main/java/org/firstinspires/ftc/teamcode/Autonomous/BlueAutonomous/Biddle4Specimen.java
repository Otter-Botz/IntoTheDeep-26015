package org.firstinspires.ftc.teamcode.Autonomous.BlueAutonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunnerlol.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.ArmSlider;
import org.firstinspires.ftc.teamcode.common.PID_Arm;
import org.firstinspires.ftc.teamcode.common.PID_Slider;
import org.firstinspires.ftc.teamcode.common.claw;
import org.firstinspires.ftc.teamcode.common.wrist;

public class Biddle4Specimen extends LinearOpMode {
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

        Pose2d initialPose = new Pose2d(-16, 70, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)

                .strafeToLinearHeading(new Vector2d(0, 36), Math.toRadians(270))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-47, 42), Math.toRadians(270))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-59, 42), Math.toRadians(270))
                .waitSeconds(1)
                .turn(-0.5)
                .waitSeconds(1)
                .turn(0.5)
                .strafeToLinearHeading(new Vector2d(-47, 50), Math.toRadians(270))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(0, 36), Math.toRadians(270))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-47, 50), Math.toRadians(270))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(0, 36), Math.toRadians(270))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-47, 50), Math.toRadians(270))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(0, 36), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-59, 42), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-59, 60), Math.toRadians(270));




        Action trajectoryActionCloseOut = tab1.fresh()
                .build();

        // actions that need to happen on init; for instance, a claw tightening.


        while (!isStopRequested() && !opModeIsActive()) {


        }


        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        tab1.build(),
                        trajectoryActionCloseOut
                )
        );
    }
}
