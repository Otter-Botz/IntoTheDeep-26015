package org.firstinspires.ftc.teamcode.Autonomous.RedAutonomous;



import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Roadrunnerlol.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.ArmSlider;
import org.firstinspires.ftc.teamcode.common.PID_Arm;
import org.firstinspires.ftc.teamcode.common.PID_Slider;
import org.firstinspires.ftc.teamcode.common.claw;
import org.firstinspires.ftc.teamcode.common.vroomVroom;
import org.firstinspires.ftc.teamcode.common.wrist;


@Disabled
public class riddle1plus3 extends LinearOpMode {

    claw claw = new claw();
    ArmSlider ArmSlider = new ArmSlider();
    PID_Arm PID_Arm = new PID_Arm();
    vroomVroom vroom = new vroomVroom();
    PID_Slider PID_Slider = new PID_Slider();
    wrist wrist = new wrist();


    @Override
    public void runOpMode() throws InterruptedException {
        claw.init(hardwareMap);
        ArmSlider.init(hardwareMap);
        PID_Arm.init(hardwareMap);
        vroom.init(hardwareMap);
        PID_Slider.init(hardwareMap);
        wrist.init(hardwareMap);
        Pose2d initialPose = new Pose2d(-72, -18, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder Riddle = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0, 38));
               // PID_Arm.up();
                PID_Slider.sliderMotor.setPower(0.5);
                PID_Slider.sliderMotorMotor.setPower(-0.5);
                PID_Slider.sliderMotor.setTargetPosition(900);
                PID_Slider.sliderMotorMotor.setTargetPosition(900);
                claw.set1();
                PID_Slider.sliderMotor.setTargetPosition(0);
                PID_Slider.sliderMotorMotor.setTargetPosition(0);


        TrajectoryActionBuilder Riddle2 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(49,36));
                claw.set2();

                PID_Arm.armMotor.setTargetPosition(500);
                PID_Slider.sliderMotor.setPower(0.5);
                PID_Slider.sliderMotorMotor.setPower(0.5);
                PID_Arm.armMotor.setPower(0.5);
                PID_Slider.sliderMotor.setTargetPosition(900);
                PID_Slider.sliderMotorMotor.setTargetPosition(-900);
                claw.set1();
                PID_Slider.sliderMotor.setTargetPosition(0);
                PID_Slider.sliderMotorMotor.setTargetPosition(0);
                PID_Arm.armMotor.setTargetPosition(0);


        TrajectoryActionBuilder Riddle3 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(60,36));
                PID_Arm.armMotor.setPower(0.5);
                PID_Slider.sliderMotor.setPower(0.5);
                PID_Slider.sliderMotorMotor.setPower(0.5);
                claw.set2();
                PID_Arm.armMotor.setTargetPosition(500);
                PID_Slider.sliderMotor.setTargetPosition(900);
                PID_Slider.sliderMotorMotor.setTargetPosition(-900);
                claw.set1();
                PID_Slider.sliderMotor.setTargetPosition(0);
                PID_Slider.sliderMotorMotor.setTargetPosition(0);
                PID_Arm.armMotor.setTargetPosition(0);

        TrajectoryActionBuilder Riddle4 = drive.actionBuilder(initialPose);
            Riddle.turnTo(Math.toRadians(90))
            .turnTo(Math.PI / 6);
            Riddle.build();
            claw.set2();
            PID_Arm.armMotor.setPower(0.5);
            PID_Slider.sliderMotor.setPower(0.5);
            PID_Slider.sliderMotorMotor.setPower(0.5);
            PID_Arm.armMotor.setTargetPosition(500);
            PID_Slider.sliderMotor.setTargetPosition(900);
            PID_Slider.sliderMotorMotor.setTargetPosition(-900);
            PID_Slider.sliderMotor.setTargetPosition(0);
            PID_Slider.sliderMotorMotor.setTargetPosition(0);
            PID_Arm.armMotor.setTargetPosition(0);

        TrajectoryActionBuilder Riddle5 = drive.actionBuilder(initialPose)
                //PARK LEFT SIDE IN WHITE AREA
                .strafeTo(new Vector2d(26,-11));
            /* PARK RIGHT SIDE IN WHITE AREA
                 .strafeTo(new Vector2d(26,11));*/

        //  park in wing
        // .strafeTo(new Vector2d(-58,62));








    }
}



