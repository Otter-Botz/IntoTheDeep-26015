package org.firstinspires.ftc.teamcode.Autonomous.BlueAutonomous;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
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
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public double open = 0;
    public double close = 0.3;

    // Slider
    public static final double IDLE_SPEED = 0.0;

    public DcMotor slideMotor;
    public DcMotor slideMotorMotor;
    public static final double SLIDER_UP_SPEED = 1.0;
    public static final double SLIDER_DOWN_SPEED = 0.5;
    public static final double SLIDER_HOLD_SPEED = 0.001;

    private final double ticks_in_degrees = 700 / 180;

    public class armSlideMotor {
        public DcMotor armSlideMotor;
        public CRServo armServo;
        //int motorTicks = 125;
        //int motorPos = 150;
        int encoderPosition = armSlideMotor.getCurrentPosition();

        public class SliderOut implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                armServo.setPower(1);
                return false;
            }
        }

        public Action slideout() {
            return new HighBasketAutoBlue.armSlideMotor.SliderOut();
        }



    }

    public class armSlide {
        public CRServo armServo;

        public armSlide(HardwareMap hardwareMap) {
            armServo = hardwareMap.get(CRServo.class, "servoSlide");
        }

        public class slideOut implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                armServo.setPower(0.5);
                return false;
            }
        }

        public Action slideout() {
            return new HighBasketAutoBlue.armSlide.slideOut();
        }

        public class armIn implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                armServo.setPower(-0.5);
                return false;
            }
        }

        public Action armSliderIN() {
            return new HighBasketAutoBlue.armSlide.armIn();
        }


        public class armWait implements Action {
            ElapsedTime time = new ElapsedTime();
            double armtime = time.seconds();
            boolean armActive = false;


            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                armServo.setPower(-1);
                sleep(1000);
                armActive = true;
                if (armActive == true) {
                    armServo.setPower(0);
                }
                return false;
            }


        }

        public Action armWaitTime() {
            return new HighBasketAutoBlue.armSlide.armWait();
        }


        public class armWaitTimeBack implements Action {
            ElapsedTime time = new ElapsedTime();
            double armtime = time.seconds();
            boolean armActive = false;


            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                armServo.setPower(1);
                sleep(1000);
                armActive = true;
                if (armActive == true) {
                    armServo.setPower(0);
                }
                return false;
            }


        }

        public Action armWaitTimeBack() {
            return new HighBasketAutoBlue.armSlide.armWaitTimeBack();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(38, 62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        org.firstinspires.ftc.teamcode.Autonomous.Common.PID_Arm armMotor = new org.firstinspires.ftc.teamcode.Autonomous.Common.PID_Arm(hardwareMap);
        autoWrist wristServo = new autoWrist(hardwareMap);
        HighBasketAutoBlue.armSlide slideServo = new HighBasketAutoBlue.armSlide(hardwareMap);
        autoClaw clawServo = new autoClaw(hardwareMap);

        waitForStart();

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

        TrajectoryActionBuilder ScoreHighBasket = drive.actionBuilder(initialPose)
                //Score Preloaded Sample
                .strafeToLinearHeading(new Vector2d(56, 55), Math.toRadians(60));

//        TrajectoryActionBuilder MoveABitForwardToScore = drive.actionBuilder(new Pose2d(Tab7X, Tab7Y, Math.toRadians(270)))
//                .strafeToLinearHeading(new Vector2d(0, 42), Math.toRadians(270));

        TrajectoryActionBuilder MoveToFirstSample = drive.actionBuilder(new Pose2d(Tab1X, Tab1Y, Math.toRadians(270)))
                //Move to first sample
                .strafeToLinearHeading(new Vector2d(53, 50), Math.toRadians(270));

        TrajectoryActionBuilder ScoreOnHighBasket1 = drive.actionBuilder(new Pose2d(Tab2X, Tab2Y, Math.toRadians(270)))
                //Pick Up and move back
                .strafeToLinearHeading(new Vector2d(60, 57), Math.toRadians(60));

        TrajectoryActionBuilder MoveToSecondSample = drive.actionBuilder(new Pose2d(Tab3X, Tab3Y, Math.toRadians(200)))
                //Move to second Sample
                .strafeToLinearHeading(new Vector2d(52, 20), Math.toRadians(180))
                .waitSeconds(0);

        TrajectoryActionBuilder ScoreOnHighBasket2 = drive.actionBuilder(new Pose2d(Tab4X, Tab4Y, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(52, 57), Math.toRadians(200))
                .waitSeconds(0);

        TrajectoryActionBuilder MoveToSample3 = drive.actionBuilder(new Pose2d(Tab5X, Tab5Y, Math.toRadians(200)))
                .strafeToLinearHeading(new Vector2d(56, 20), Math.toRadians(180))
                .waitSeconds(0);

        TrajectoryActionBuilder ScoreOnHighBasket3 = drive.actionBuilder(new Pose2d(Tab6X, Tab6Y, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(56, 57), Math.toRadians(200))
                .waitSeconds(0);
        Action trajectoryActionCloseOut = ScoreOnHighBasket3.fresh()
                .strafeToLinearHeading(new Vector2d(39, 10), Math.toRadians(270))
                .build();


        Actions.runBlocking(
                new SequentialAction(

                        ScoreHighBasket.build()
                        //Wrist
                        //Arm
                        //Main Sliders
                        //Claw Open
                        // MoveToFirstSample.build()
                        //Change Claw and Wrist Back to intake position


//                        new ParallelAction(
//                                armMotor.mathRun(),
//                                new SequentialAction(
////                                        wristServo.wristUp(),
////                                        .build()
//                                        // parkCloseOut
//
//                                )
//                        )

                        /*
                        armMotor.armDown(),
                        clawServo.clawClose(),
                        armMotor.backDown(),
                        clawServo.clawOpen()
                        */





                )


        );
        // armMotor.armUp(),
                        /*
                        clawServo.clawOpen(),
                        armMotor.armDown(),
                        clawServo.clawClose(),
                         armMotor.backDown(),
                        clawServo.clawOpen(),
                        sampleTransfer2.build(),
                        armMotor.armDown(),
                        clawServo.clawClose(),
                        armMotor.backDown(),
                        clawServo.clawOpen(),
                        sampleTransfer3.build(),
                         armMotor.armDown(),
                        clawServo.clawClose(),
                         armMotor.backDown(),
                         clawServo.clawOpen(),
                        specimenPickup1.build(),
                        armMotor.backDown(),
                        clawServo.clawClose(),
                        armMotor.armUp(),
                        score1Pickup2.build(),
                        clawServo.clawOpen(),
                         armMotor.backDown(),
                        clawServo.clawClose(),
                         armMotor.armUp(),
                        score2Pickup3.build(),
                        clawServo.clawOpen(),
                        armMotor.backDown(),
                       clawServo.clawClose(),
                       armMotor.armUp(),
                        score3.build(),

                         */



    }}