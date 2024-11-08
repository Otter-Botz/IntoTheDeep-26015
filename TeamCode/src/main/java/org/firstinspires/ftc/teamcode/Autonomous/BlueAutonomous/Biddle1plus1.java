package org.firstinspires.ftc.teamcode.Autonomous.BlueAutonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Roadrunnerlol.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.ArmSlider;
import org.firstinspires.ftc.teamcode.common.PID_Arm;
import org.firstinspires.ftc.teamcode.common.Slider;
import org.firstinspires.ftc.teamcode.common.claw;
import org.firstinspires.ftc.teamcode.common.wrist;

public class Biddle1plus1 {

    @Disabled
    public class BlueAuton1specimenand1baskets extends LinearOpMode {
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
            Servo clawServo = hardwareMap.get(Servo.class, "clawServo");

            waitForStart();

            Pose2d initialPose = new Pose2d(-72, -18, Math.toRadians(0));
            MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

            TrajectoryActionBuilder BlueAutonomousDrive = drive.actionBuilder(initialPose);

            // Robot moves to the specified x coordinate in the direction of the robot heading (straight line).
            // Both `lineToX()` and `lineToXConstantHeading()` are equivalent.
            // 🚨 Will cause an error if your heading is perpendicular to direction your robot is traveling! 🚨
            BlueAutonomousDrive.lineToX(46)
                    .lineToXConstantHeading(46)
                    .build();
            //Slider Up
            PID_Arm.armMotor.setPower(0.4);
            PID_Arm.armMotor.setTargetPosition(300);

            claw.set1();
            //Claw Open
            claw.set2();
            //Claw Close

            PID_Arm.armMotor.setTargetPosition(-300);

            BlueAutonomousDrive.lineToXConstantHeading(-8)
            // Robot moves to the specified coordinates while linearly interpolating between the start heading and a specified end heading
            // In other words, it constantly turns to a certain heading (once more, in radians) while moving to the specified coordinates.
            .strafeToLinearHeading(new Vector2d(36, 36), Math.toRadians(90))
            .build();
            //Claw Open
            claw.set1();
            //Claw Close
            claw.set2();

            // Robot moves to the specified coordinates in a spline path while following a tangent heading interpolator
            BlueAutonomousDrive.splineTo(new Vector2d(48, 48), Math.PI / 2)
            .build();
            PID_Slider.sliderMotor.setPower(0.5);
            PID_Slider.sliderMotorMotor.setPower(0.5);
            PID_Slider.sliderMotor.setTargetPosition(1250);
            PID_Slider.sliderMotorMotor.setTargetPosition(1250);

            //Claw Open
            claw.set1();
            //Claw Close
            claw.set2();

            PID_Slider.sliderMotor.setTargetPosition(-1250);
            PID_Slider.sliderMotorMotor.setTargetPosition(-1250);


        }
    }
}
