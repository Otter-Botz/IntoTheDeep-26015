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
import org.firstinspires.ftc.teamcode.common.PID_Slider;
import org.firstinspires.ftc.teamcode.common.claw;
import org.firstinspires.ftc.teamcode.common.wrist;

public class BlueAutonSpecimenOnly {
    public class BlueAuton1Speciemenand1basket {

        @Disabled
        public class BlueAuton1specimenand3baskets extends LinearOpMode {
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
                //Slider Up
                PID_Arm.armMotor.setPower(0.4);
                PID_Arm.armMotor.setTargetPosition(300);

                claw.set1();
                //Claw Open
                claw.set2();
                //Claw Close

                PID_Arm.armMotor.setTargetPosition(-300);

                BlueAutonomousDrive.lineToXConstantHeading(-8);
                // Robot moves to the specified coordinates while linearly interpolating between the start heading and a specified end heading
                // In other words, it constantly turns to a certain heading (once more, in radians) while moving to the specified coordinates.
                BlueAutonomousDrive.strafeToLinearHeading(new Vector2d(36, 36), Math.toRadians(90));
                BlueAutonomousDrive.build();

            }
        }
    }
}
