package org.firstinspires.ftc.teamcode.Autonomous.BlueAutonomous;

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
        public class BlueAuton1SpecimenOnly extends LinearOpMode {
            claw claw = new claw();
            ArmSlider ArmSlider = new ArmSlider();
            PID_Arm PID_Arm = new PID_Arm();
            Slider PID_Slider = new Slider();
            wrist wrist = new wrist();

            @Override
            public void runOpMode() throws InterruptedException {

                Pose2d initialPose = new Pose2d(-72, -18, Math.toRadians(0));
                MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

                claw.init(hardwareMap);
                ArmSlider.init(hardwareMap);
                PID_Arm.init(hardwareMap);
                PID_Slider.init(hardwareMap);
                wrist.init(hardwareMap);
                Servo clawServo = hardwareMap.get(Servo.class, "clawServo");

                waitForStart();

                TrajectoryActionBuilder BlueAutonSpecimen = drive.actionBuilder(initialPose)
                        .strafeTo(new Vector2d(0, 38))
                        // Robot moves to the specified x coordinate in the direction of the robot heading (straight line).
                        // Both `lineToX()` and `lineToXConstantHeading()` are equivalent.
                        // 🚨 Will cause an error if your heading is perpendicular to direction your robot is traveling! 🚨
                        .lineToX(48)
                        .lineToXConstantHeading(48)
                                .endTrajectory();
                //PID_Arm.up();
                PID_Slider.sliderMotor.setPower(0.5);
                PID_Slider.sliderMotorMotor.setPower(-0.5);
                PID_Slider.sliderMotor.setTargetPosition(900);
                PID_Slider.sliderMotorMotor.setTargetPosition(900);
                claw.set1();
                PID_Slider.sliderMotor.setTargetPosition(0);
                PID_Slider.sliderMotorMotor.setTargetPosition(0);
                //hook specimen on rung

            }
        }


