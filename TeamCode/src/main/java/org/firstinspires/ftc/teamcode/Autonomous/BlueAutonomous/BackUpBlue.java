package org.firstinspires.ftc.teamcode.Autonomous.BlueAutonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Roadrunnerlol.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.ArmSlider;
import org.firstinspires.ftc.teamcode.common.PID_Arm;
import org.firstinspires.ftc.teamcode.common.Slider;
import org.firstinspires.ftc.teamcode.common.claw;
import org.firstinspires.ftc.teamcode.common.wrist;

@Autonomous
public class BackUpBlue extends LinearOpMode {
    org.firstinspires.ftc.teamcode.common.claw claw = new claw();
    org.firstinspires.ftc.teamcode.common.ArmSlider ArmSlider = new ArmSlider();
    org.firstinspires.ftc.teamcode.common.PID_Arm PID_Arm = new PID_Arm();
    Slider PID_Slider = new Slider();
    org.firstinspires.ftc.teamcode.common.wrist wrist = new wrist();
    public static double p = 0.005, i = 0.03, d = 0.0005;
    public static double f = 0.12;

    public double target = -750;

    private final double ticks_in_degrees = 700 / 180;

    // auto arm stuff
    public class autoArm {
        private PIDController controller;
        public DcMotor armMotor;

        //init
        public autoArm(HardwareMap hardwareMap) {
            armMotor = hardwareMap.get(DcMotor.class, "armMotor");
            controller = new PIDController(p, i, d);
        }

        //arm  up stuff
        public class up implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                target = -65;
                return false;
            }

        }
        public Action armUp(){
            return new BackUpBlue.autoArm.up();
        }

        // arm all the way down
        public class down implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                target = -700;
                return false;
            }

        }
        public Action armDown(){
            return new BackUpBlue.autoArm.down();
        }


        public class backDown implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                target = -200;
                return false;
            }

        }
        public Action backDown(){
            return new BackUpBlue.autoArm.backDown();
        }


        //math for PID
        public void math() {


            controller.setPID(p, i , d);
            int slidePos = armMotor.getCurrentPosition();
            double pid = controller.calculate(slidePos, target);
            double ff = Math.cos(Math.toRadians(target/ ticks_in_degrees)) * f;
            double power = pid + ff;
            armMotor.setPower(power);
        }
    }



    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(-16, 62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        claw.init(hardwareMap);
        ArmSlider.init(hardwareMap);
        PID_Arm.init(hardwareMap);
        PID_Slider.init(hardwareMap);
        wrist.init(hardwareMap);
        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");

        waitForStart();

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                //Score Preloaded Specimen
                .strafeToLinearHeading(new Vector2d(0, 38), Math.toRadians(270))
                .waitSeconds(1)


                //Sample One
                .strafeToLinearHeading(new Vector2d(-48, 38), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-44, 5), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-60, 5), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-60, 55), Math.toRadians(270))
                .waitSeconds(1)

        //Second Sample
                .strafeToLinearHeading(new Vector2d(-60, 5), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-72, 5), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-72, 55), Math.toRadians(270))
                .waitSeconds(1)

//              Score 1
                .strafeToLinearHeading(new Vector2d(-57, 55), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(0, 38), Math.toRadians(270))
                .waitSeconds(1)

        //Score 2
                .strafeToLinearHeading(new Vector2d(-57, 55), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(0, 38), Math.toRadians(270))
                .waitSeconds(1);


//                .strafeToLinearHeading(new Vector2d(-55, 10), Math.toRadians(270))
//                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(-55, 62), Math.toRadians(270))
//                .waitSeconds(2)
//
//                //.strafeToLinearHeading(new Vector2d(-55, 10), Math.toRadians(270))
//                //.waitSeconds(2)
////                .strafeToLinearHeading(new Vector2d(-62, 10), Math.toRadians(270))
////                .waitSeconds(2)
////                .strafeToLinearHeading(new Vector2d(-62, 62), Math.toRadians(270))
////                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(0, 62), Math.toRadians(270))
//                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(0, 38), Math.toRadians(270))
//                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(0, 60), Math.toRadians(270))
//                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(-55, 60), Math.toRadians(270))
//                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(0, 60), Math.toRadians(270))
//                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(0, 38), Math.toRadians(270))
//                .waitSeconds(2);
        Action trajectoryActionCloseOut = tab1.fresh()
                .strafeTo(new Vector2d(-59, 50))
                .build();


        Actions.runBlocking(
                new SequentialAction(
                        tab1.build(),
                         trajectoryActionCloseOut
                )
        );


    }
}
