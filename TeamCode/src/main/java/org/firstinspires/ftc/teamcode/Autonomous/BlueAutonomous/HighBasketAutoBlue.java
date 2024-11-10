package org.firstinspires.ftc.teamcode.Autonomous.BlueAutonomous;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunnerlol.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.ArmSlider;
import org.firstinspires.ftc.teamcode.common.PID_Arm;
import org.firstinspires.ftc.teamcode.common.Slider;
import org.firstinspires.ftc.teamcode.common.claw;
import org.firstinspires.ftc.teamcode.common.wrist;

@Autonomous
public class HighBasketAutoBlue extends LinearOpMode {
    org.firstinspires.ftc.teamcode.common.claw claw = new claw();
    org.firstinspires.ftc.teamcode.common.ArmSlider ArmSlider = new ArmSlider();
    org.firstinspires.ftc.teamcode.common.PID_Arm PID_Arm = new PID_Arm();
    Slider PID_Slider = new Slider();
    org.firstinspires.ftc.teamcode.common.wrist wrist = new wrist();
    public static double
            p = 0.005,
            i = 0.03,
            d = 0.0005;
    public static double f = 0.12;
    public double target = -750;

    // Slider
    public static final double IDLE_SPEED = 0.0;

    public DcMotor slideMotor;
    public DcMotor slideMotorMotor;
    public static final double SLIDER_UP_SPEED = 1.0;
    public static final double SLIDER_DOWN_SPEED = 0.5;
    public static final double SLIDER_HOLD_SPEED = 0.001;

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
            return new HighBasketAutoBlue.autoArm.up();
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
            return new HighBasketAutoBlue.autoArm.down();
        }


        public class backDown implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                target = -200;
                return false;
            }

        }
        public Action backDown(){
            return new HighBasketAutoBlue.autoArm.backDown();
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
    public void setSliderIdlePosition() {
        slideMotor.setPower(IDLE_SPEED);
        slideMotorMotor.setPower(IDLE_SPEED);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void  LowBaskets (){
        slideMotor.setTargetPosition(1350);
        slideMotorMotor.setTargetPosition(1350);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(SLIDER_UP_SPEED);
        slideMotorMotor.setPower(SLIDER_UP_SPEED);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setSliderIdlePosition();
    }
    public void  HighRung (){
        slideMotor.setTargetPosition(1750);
        slideMotorMotor.setTargetPosition(1750);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(SLIDER_UP_SPEED);
        slideMotorMotor.setPower(SLIDER_UP_SPEED);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setSliderIdlePosition();
    }
    public void  LowRung (){
        slideMotor.setTargetPosition(1350);
        slideMotorMotor.setTargetPosition(1350);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(SLIDER_UP_SPEED);
        slideMotorMotor.setPower(SLIDER_UP_SPEED);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setSliderIdlePosition();
    }
    public void  HighBaskets (){
        slideMotor.setTargetPosition(1750);
        slideMotorMotor.setTargetPosition(1750);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(SLIDER_UP_SPEED);
        slideMotorMotor.setPower(SLIDER_UP_SPEED);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setSliderIdlePosition();
    }
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(38, 62, Math.toRadians(270));
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
                .waitSeconds(2)

                //Move to first sample
                .strafeToLinearHeading(new Vector2d(56, 38), Math.toRadians(270))
                .waitSeconds(2)
                //Pick Up and move back
                .strafeToLinearHeading(new Vector2d(60, 57), Math.toRadians(200))
                .waitSeconds(2)
                //Move to second Sample
                .strafeToLinearHeading(new Vector2d(52, 20), Math.toRadians(180))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(52, 57), Math.toRadians(200))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(56, 20), Math.toRadians(180))
                .waitSeconds(0.5)
                //arm
                .strafeToLinearHeading(new Vector2d(56, 57), Math.toRadians(200))
                .waitSeconds(2);

        Action trajectoryActionCloseOut = tab1.fresh()
                .strafeToLinearHeading(new Vector2d(39, 10), Math.toRadians(270))
                .build();


        Actions.runBlocking(
                new SequentialAction(
                        tab1.build(),
                        trajectoryActionCloseOut
                )
        );









    }}