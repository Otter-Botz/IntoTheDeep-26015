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
import org.firstinspires.ftc.teamcode.Autonomous.Common.autoClaw;
import org.firstinspires.ftc.teamcode.Autonomous.Common.autoWrist;
import org.firstinspires.ftc.teamcode.Roadrunnerlol.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.ArmSlider;
import org.firstinspires.ftc.teamcode.common.PID_Arm;
import org.firstinspires.ftc.teamcode.common.Slider;
import org.firstinspires.ftc.teamcode.common.claw;
import org.firstinspires.ftc.teamcode.common.wrist;


@Autonomous
public class RedParkingLeftSide extends LinearOpMode {
    org.firstinspires.ftc.teamcode.common.claw claw = new claw();
    org.firstinspires.ftc.teamcode.common.ArmSlider ArmSlider = new ArmSlider();
    org.firstinspires.ftc.teamcode.common.PID_Arm PID_Arm = new PID_Arm();
    Slider PID_Slider = new Slider();
    org.firstinspires.ftc.teamcode.common.wrist wrist = new wrist();

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(-38, -62, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        autoClaw autoClaw = new autoClaw(hardwareMap);
        // armSlide armMotor = new armSlide(hardwareMap);
        autoWrist wristServo = new autoWrist(hardwareMap);
        // armSlide slideServo = new armSlide(hardwareMap);
        autoClaw clawServo = new autoClaw(hardwareMap);
        AutoMainSliders AutoMainSliders = new AutoMainSliders(hardwareMap);



        double Tab1X = 0;
        double Tab1Y = 38;
        double Tab2X = 56;
        double Tab2Y = 38;
        double Tab3X = 60;
        double Tab3Y = 57;
        double Tab4X = 52;
        double Tab4Y = 20;
        double Tab5X = 52;
        double Tab5Y = 57;
        double Tab6X = 56;
        double Tab6Y = 20;
        double Tab7X = 0;
        double Tab7Y = 38;

        waitForStart();

        //Park = (39, 10)
        TrajectoryActionBuilder MoveToPark = drive.actionBuilder(initialPose)
                //Park
                .strafeToLinearHeading(new Vector2d(-39, -60), Math.toRadians(90));
        Action trajectoryActionCloseOut = MoveToPark.fresh()
                .strafeToLinearHeading(new Vector2d(-35, -15), Math.toRadians(90))
                .build();


        Actions.runBlocking(
                new SequentialAction(
                        MoveToPark.build(),
                        trajectoryActionCloseOut
                ));

    }

}



 */