package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Autonomous.PinPoint.AutoMechanisms;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class pedroBlueSpec extends OpMode {
    private Follower follower;

    public Motor armMotor;
    public DcMotor sliderMotor;
    public DcMotor sliderMotorMotor;
    public Servo clawServo;
    public Servo wristServo;
    public DcMotor frontleftmotor;
    public DcMotor backleftmotor;
    public DcMotor frontrightmotor;
    public DcMotor backrightmotor;

    AutoMechanisms mechanisms = new AutoMechanisms();

    //start position
    private final Pose startPose = new Pose(10,60, Math.toRadians(0));

    // place preload
    private final Pose preload = new Pose(35,70.4, Math.toRadians(0));

    //push first sample into observation zone
    private final Pose Move1 = new Pose(29.5, 40.6, Math.toRadians(0));
    private final Pose Move2 = new Pose(60,40.6, Math.toRadians(0));
    private final Pose Move3 = new Pose(60, 26.7, Math.toRadians(0));
    private final Pose pushFirstSample = new Pose(10, 26.7, Math.toRadians(0));

    //Push second sample into observation zone
    private final Pose move1 = new Pose(60,15, Math.toRadians(0));
    private final Pose pushSecondSample  = new Pose(10,15, Math.toRadians(0));

    //push third sample into observation zone
    private final Pose move2 = new Pose(60,9, Math.toRadians(0));
    private final Pose pushThirdSample = new Pose(9.9,9, Math.toRadians(0));

    //wait for hp and pickup 1
    private final Pose waitForHp = new Pose(22,9, Math.toRadians(0));

    //drive to rung and score 1
    private final Pose score = new Pose(35,70.4, Math.toRadians(0));

    private Path scorePreload,park;
    private PathChain MoveOne, MoveTwo, MoveThree, PushFirstSample, comeBack1, moveOne, PushSecondSample, ComeBack2, moveTwo, PushThirdSample, WaitForHP, pickup1, Score1, pickup2, score2, pickup3, score3;

    public void buildPaths(){
        //start pose ---> score preload
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(preload)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), preload.getHeading());

        MoveOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preload), new Point(Move1)))
                .setLinearHeadingInterpolation(preload.getHeading(), Move1.getHeading())
                .build();

        MoveTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Move1), new Point(Move2)))
                .setLinearHeadingInterpolation(Move1.getHeading(), Move2.getHeading())
                .build();

        MoveThree = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Move2), new Point(Move3)))
                .setLinearHeadingInterpolation(Move2.getHeading(), Move3.getHeading())
                .build();

        PushFirstSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Move3), new Point(pushFirstSample)))
                .setLinearHeadingInterpolation(Move3.getHeading(), pushFirstSample.getHeading())
                .build();

        comeBack1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushFirstSample), new Point(Move3)))
                .setLinearHeadingInterpolation(pushFirstSample.getHeading(), Move3.getHeading())
                .build();

        moveOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Move3), new Point(move1)))
                .setLinearHeadingInterpolation(Move3.getHeading(), move1.getHeading())
                .build();

        PushSecondSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(move1), new Point(pushSecondSample)))
                .setLinearHeadingInterpolation(move1.getHeading(), pushSecondSample.getHeading())
                .build();

        ComeBack2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushSecondSample), new Point(move1)))
                .setLinearHeadingInterpolation(pushSecondSample.getHeading(), move1.getHeading())
                .build();

        moveTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(move1), new Point(move2)))
                .setLinearHeadingInterpolation(move1.getHeading(), move2.getHeading())
                .build();

        PushThirdSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(move2), new Point(pushThirdSample)))
                .setLinearHeadingInterpolation(move2.getHeading(), pushThirdSample.getHeading())
                .build();

        WaitForHP = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushThirdSample), new Point(waitForHp)))
                .setLinearHeadingInterpolation(pushThirdSample.getHeading(), waitForHp.getHeading())
                .build();

        pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(waitForHp), new Point(pushThirdSample)))
                .setLinearHeadingInterpolation(waitForHp.getHeading(), pushThirdSample.getHeading())
                .build();

        Score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushThirdSample), new Point(score)))
                .setLinearHeadingInterpolation(pushFirstSample.getHeading(), score.getHeading())
                .build();

        pickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score), new Point(pushThirdSample)))
                .setLinearHeadingInterpolation(score.getHeading(), pushThirdSample.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushThirdSample), new Point(score)))
                .setLinearHeadingInterpolation(pushThirdSample.getHeading(), score.getHeading())
                .build();

        pickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score), new Point(pushThirdSample)))
                .setLinearHeadingInterpolation(score.getHeading(), pushThirdSample.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushThirdSample), new Point(score)))
                .setLinearHeadingInterpolation(pushThirdSample.getHeading(), score.getHeading())
                .build();

        park = new Path(new BezierLine(new Point(score), new Point(pushThirdSample)));
                park.setLinearHeadingInterpolation(score.getHeading(), pushThirdSample.getHeading());


    }
    //yayayay

       @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
