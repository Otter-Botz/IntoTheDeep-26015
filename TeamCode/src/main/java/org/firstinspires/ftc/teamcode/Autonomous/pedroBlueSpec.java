package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.ArmSlider;
import org.firstinspires.ftc.teamcode.common.PID_Arm;
import org.firstinspires.ftc.teamcode.common.Slider;
import org.firstinspires.ftc.teamcode.common.claw;
import org.firstinspires.ftc.teamcode.common.wrist;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class pedroBlueSpec extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;


    ArmSlider armslider = new ArmSlider();
    Slider slider = new Slider();
    claw claw = new claw();
    wrist wrist = new wrist();
    PID_Arm arm = new PID_Arm();





    //start position
    private final Pose startPose = new Pose(10,60, Math.toRadians(0));


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
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(score)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), score.getHeading());

        MoveOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score), new Point(Move1)))
                .setLinearHeadingInterpolation(score.getHeading(), Move1.getHeading())
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

    public void autonomousPathUpdate(){
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (follower.getPose().getX() > (score.getX() - 1) && follower.getPose().getY() > (score.getY() - 1)){
                    arm.specimen();
                    claw.AutoOpen();
                    follower.followPath(scorePreload,true);
                    setPathState(2);
                }
                break;
            case 2:
                if (follower.getPose().getX() > (Move1.getX() - 1) && follower.getPose().getY() > (Move1.getY() - 1)){
                    arm.down();
                    follower.followPath(MoveOne,true);
                    setPathState(3);
                }
                break;
            case 3:
                if (follower.getPose().getX() > (Move2.getX() - 1) && follower.getPose().getY() > (Move2.getY() - 1)){
                    follower.followPath(MoveTwo,true);
                    setPathState(4);
                }
                break;
            case 4:
                if (follower.getPose().getX() > (Move3.getX() - 1) && follower.getPose().getY() > (Move3.getY() - 1)){
                    follower.followPath(MoveThree,true);
                    setPathState(5);
                }
                break;
            case 5:
                if (follower.getPose().getX() > (pushFirstSample.getX() - 1) && follower.getPose().getY() > (pushFirstSample.getY() - 1)){
                    follower.followPath(PushFirstSample,true);
                    setPathState(6);
                }
                break;
            case 6:
                if (follower.getPose().getX() > (Move3.getX() - 1) && follower.getPose().getY() > (Move3.getY() - 1)){
                    follower.followPath(comeBack1,true);
                    setPathState(7);
                }
                break;
            case 7:
                if (follower.getPose().getX() > (move1.getX() - 1) && follower.getPose().getY() > (move1.getY() - 1)){
                    follower.followPath(moveOne,true);
                    setPathState(8);
                }
                break;
            case 8:
                if (follower.getPose().getX() > (pushSecondSample.getX() - 1) && follower.getPose().getY() > (pushSecondSample.getY() - 1)){
                    follower.followPath(PushSecondSample,true);
                    setPathState(9);
                }
                break;
            case 9:
                if (follower.getPose().getX() > (move1.getX() - 1) && follower.getPose().getY() > (move1.getY() - 1)){
                    follower.followPath(ComeBack2,true);
                    setPathState(10);
                }
                break;
            case 10:
                if (follower.getPose().getX() > (move2.getX() - 1) && follower.getPose().getY() > (move2.getY() - 1)){
                    follower.followPath(moveTwo,true);
                    setPathState(11);
                }


                }
    }

    public void setPathState(int pState){
        pathState = pState;
        pathTimer.resetTimer();
    }

       @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();


    }

    @Override
    public void loop() {

    }

    public void start(){
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}
