package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.TrajectoryBuilder;

import java.security.cert.CertPathBuilder;

@Autonomous(name= "Blight" )
public class Blight extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

    }
}



/*

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d(72, 36, 0))
                .strafeRight(10)
                .forward(5)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(myTrajectory);

 */





