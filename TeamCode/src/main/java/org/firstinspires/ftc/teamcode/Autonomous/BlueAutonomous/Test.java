package org.firstinspires.ftc.teamcode.Autonomous.BlueAutonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunnerlol.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.ArmSlider;
import org.firstinspires.ftc.teamcode.common.PID_Arm;
import org.firstinspires.ftc.teamcode.common.Slider;
import org.firstinspires.ftc.teamcode.common.claw;
import org.firstinspires.ftc.teamcode.common.wrist;

    @Autonomous
    public class Test extends LinearOpMode {
        claw claw = new claw();
        ArmSlider ArmSlider = new ArmSlider();
        PID_Arm PID_Arm = new PID_Arm();
        Slider PID_Slider = new Slider();
        wrist wrist = new wrist();

        @Override
        public void runOpMode() throws InterruptedException {

            claw.init(hardwareMap);
            ArmSlider.init(hardwareMap);
            PID_Arm.init(hardwareMap);
            PID_Slider.init(hardwareMap);
            wrist.init(hardwareMap);

            Pose2d initialPose = new Pose2d(72, 9, Math.toRadians(180));
            MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


            TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                    .strafeToLinearHeading(new Vector2d(36, 0), Math.toRadians(180))
                    .waitSeconds(3)
                    .strafeToLinearHeading(new Vector2d(36, 36), Math.toRadians(180))
                    .waitSeconds(3);



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