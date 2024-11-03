package org.firstinspires.ftc.teamcode.Autonomous.BlueAutonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Roadrunnerlol.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.ArmSlider;
import org.firstinspires.ftc.teamcode.common.PID_Arm;
import org.firstinspires.ftc.teamcode.common.PID_Slider;
import org.firstinspires.ftc.teamcode.common.claw;
import org.firstinspires.ftc.teamcode.common.wrist;

    @Autonomous
    public class Test extends LinearOpMode {
        org.firstinspires.ftc.teamcode.common.claw claw = new claw();
        org.firstinspires.ftc.teamcode.common.ArmSlider ArmSlider = new ArmSlider();
        org.firstinspires.ftc.teamcode.common.PID_Arm PID_Arm = new PID_Arm();
        org.firstinspires.ftc.teamcode.common.PID_Slider PID_Slider = new PID_Slider();
        org.firstinspires.ftc.teamcode.common.wrist wrist = new wrist();

        @Override
        public void runOpMode() throws InterruptedException {

            claw.init(hardwareMap);
            ArmSlider.init(hardwareMap);
            PID_Arm.init(hardwareMap);
            PID_Slider.init(hardwareMap);
            wrist.init(hardwareMap);
            Servo clawServo = hardwareMap.get(Servo.class, "clawServo");

            waitForStart();

            Pose2d initialPose = new Pose2d(-72, -18, Math.toRadians(0));
            MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

            TrajectoryActionBuilder BlueAutonomousDrive = drive.actionBuilder(initialPose);

            // Robot moves to the specified x coordinate in the direction of the robot heading (straight line).
            // Both `lineToX()` and `lineToXConstantHeading()` are equivalent.
            // 🚨 Will cause an error if your heading is perpendicular to direction your robot is traveling! 🚨
            BlueAutonomousDrive.lineToX(46)
                    .lineToXConstantHeading(46);
            BlueAutonomousDrive.build();

        }
    }