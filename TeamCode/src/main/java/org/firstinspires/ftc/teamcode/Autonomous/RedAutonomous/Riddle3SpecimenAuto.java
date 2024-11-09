package org.firstinspires.ftc.teamcode.Autonomous.RedAutonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Roadrunnerlol.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.ArmSlider;
import org.firstinspires.ftc.teamcode.common.PID_Arm;
import org.firstinspires.ftc.teamcode.common.Slider;
import org.firstinspires.ftc.teamcode.common.claw;
import org.firstinspires.ftc.teamcode.common.wrist;


@Autonomous
public class Riddle3SpecimenAuto extends LinearOpMode {
    claw claw = new claw();
    ArmSlider ArmSlider = new ArmSlider();
    PID_Arm PID_Arm = new PID_Arm();
    Slider PID_Slider = new Slider();
    wrist wrist = new wrist();

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(0, -70, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        claw.init(hardwareMap);
        ArmSlider.init(hardwareMap);
        PID_Arm.init(hardwareMap);
        PID_Slider.init(hardwareMap);
        wrist.init(hardwareMap);
        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");

        waitForStart();

        TrajectoryActionBuilder BlueAutonSpecimen = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(0, -38), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(35, -38), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(47, -15), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(47, -62), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(47, -18), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(55, -18), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(55, -62), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(55, -18), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(62, -18), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(62, -62), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(0, -62), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(0, -38), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(0, -60), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(55, -60), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(0, -60), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(0, -38), Math.toRadians(270))
                .waitSeconds(2);

    }}