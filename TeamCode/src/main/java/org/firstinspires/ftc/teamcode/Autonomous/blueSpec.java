package org.firstinspires.ftc.teamcode.Autonomous;


import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.common.PID_Arm;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.LConstants;


/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous
public class blueSpec extends OpMode {
        PID_Arm arm = new PID_Arm();
        private Follower follower;
        private Timer pathTimer, actionTimer, opmodeTimer;

        /**
         * This is the variable where we store the state of our auto.
         * It is used by the pathUpdate method.
         */
        private int pathState;

        /* Create and Define Poses + Paths
         * Poses are built with three constructors: x, y, and heading (in Radians).
         * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
         * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
         * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
         * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
         * Lets assume our robot is 18 by 18 inches
         * Lets assume the Robot is facing the human player and we want to score in the bucket */

        /**
         * Start Pose of our robot
         */
        private final Pose startPose = new Pose(7.5, 63.3, Math.toRadians(0));

        /**
         * Scoring specimen preload
         */
        private final Pose scorePose = new Pose(35, 70.4, Math.toRadians(0));
        private final Pose scorePoseTurned = new Pose(35, 70.4, Math.toRadians(180));


        /**
         * Bezier Curve Points to move to push first sample
         */
        private final Pose moveToPushPose = new Pose(60, 36, Math.toRadians(0));
        private final Pose moveToPushControl = new Pose(18, 24, Math.toRadians(0));


        /**
         * Lining up to first sample push
         */
        private final Pose lineToSampleOnePose = new Pose(60, 29, Math.toRadians(0));

        /**
         * Pushing first sample
         */
        private final Pose pushSampleOnePose = new Pose(15, 29, Math.toRadians(0));

        /**
         * Line to sample 2
         */

        private final Pose lineToSampleTwoPose = new Pose(60, 18, Math.toRadians(0));
        /**
         * push sample 2
         */
        private final Pose  pushSampleTwoPose = new Pose(15, 18, Math.toRadians(0));

        /**
         * Line to sample 3
         */
        private final Pose  lineToSampleThreePose = new Pose(60, 10, Math.toRadians(0));
        /**
         * Push sample 3
         */
        private final Pose pushThirdSamplePose = new Pose(19, 10, Math.toRadians(0));
        /**
         * Pick up from hp postion
         */
        private final Pose pickUpPose = new Pose(13, 29, Math.toRadians(0));
        /**
         * Bezier curve points to cycle specimen
         */
        private final Pose cycleControlPose = new Pose(13, 70.4, Math.toRadians(0));
        /**
         * Park
         */
        private final Pose parkPose = new Pose(13, 29, Math.toRadians(180));



        /* These are our Paths and PathChains that we will define in buildPaths() */
        private Path scorePreload, park;
        private PathChain goToSamples, lineToSampleOne, pushSampleOne, comeBackOne, lineToSampleTwo, pushSampleTwo, comeBackTwo, lineToSampleThree, pushSampleThree, pickUpSpecOne, scoreSpec, goBackPickUp;

        /**
         * Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
         * It is necessary to do this so that all the paths are built before the auto starts.
         **/
        public void buildPaths() {

                /* There are two major types of paths components: BezierCurves and BezierLines.
                 *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
                 *    - Control points manipulate the curve between the start and end points.
                 *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
                 *    * BezierLines are straight, and require 2 points. There are the start and end points.
                 * Paths have can have heading interpolation: Constant, Linear, or Tangential
                 *    * Linear heading interpolation:
                 *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
                 *    * Constant Heading Interpolation:
                 *    - Pedro will maintain one heading throughout the entire path.
                 *    * Tangential Heading Interpolation:
                 *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
                 * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
                 * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

                /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
                scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
                scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

                /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
                goToSamples = follower.pathBuilder()
                        .addPath(new BezierCurve(new Point(scorePose), new Point(moveToPushControl), new Point(moveToPushPose)))
                        .setLinearHeadingInterpolation(scorePose.getHeading(), moveToPushPose.getHeading())
                        .build();

                /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
                lineToSampleOne = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(moveToPushPose), new Point(lineToSampleOnePose)))
                        .setLinearHeadingInterpolation(moveToPushPose.getHeading(), lineToSampleOnePose.getHeading())
                        .build();

                /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
                pushSampleOne = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(lineToSampleOnePose), new Point(pushSampleOnePose)))
                        .setLinearHeadingInterpolation(lineToSampleOnePose.getHeading(), pushSampleOnePose.getHeading())
                        .build();

                /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
                comeBackOne = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(pushSampleOnePose), new Point(lineToSampleOnePose)))
                        .setLinearHeadingInterpolation(pushSampleOnePose.getHeading(), lineToSampleOnePose.getHeading())
                        .build();

                /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
                lineToSampleTwo = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(lineToSampleOnePose), new Point(lineToSampleTwoPose)))
                        .setLinearHeadingInterpolation(lineToSampleOnePose.getHeading(), lineToSampleTwoPose.getHeading())
                        .build();

                /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
                pushSampleTwo = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(lineToSampleTwoPose), new Point(pushSampleTwoPose)))
                        .setLinearHeadingInterpolation(lineToSampleTwoPose.getHeading(), pushSampleTwoPose.getHeading())
                        .build();
                comeBackTwo = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(pushSampleTwoPose), new Point(lineToSampleTwoPose)))
                        .setLinearHeadingInterpolation(pushSampleTwoPose.getHeading(), lineToSampleTwoPose.getHeading())
                        .build();
                lineToSampleThree = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(lineToSampleTwoPose), new Point(lineToSampleThreePose)))
                        .setLinearHeadingInterpolation(lineToSampleTwoPose.getHeading(),lineToSampleThreePose.getHeading())
                        .build();
                pushSampleThree= follower.pathBuilder()
                        .addPath(new BezierLine(new Point(lineToSampleThreePose), new Point(pushThirdSamplePose)))
                        .setLinearHeadingInterpolation(lineToSampleThreePose.getHeading(),pushThirdSamplePose.getHeading())
                        .build();
                pickUpSpecOne = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(pushThirdSamplePose), new Point(pickUpPose)))
                        .setLinearHeadingInterpolation(pushThirdSamplePose.getHeading(),pickUpPose.getHeading())
                        .build();
                scoreSpec = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(pickUpPose), new Point(scorePoseTurned)))
                        .setLinearHeadingInterpolation(pickUpPose.getHeading(), scorePoseTurned.getHeading())
                        .build();
                goBackPickUp  = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(scorePoseTurned), new Point(pickUpPose)))
                        .setLinearHeadingInterpolation(scorePoseTurned.getHeading(), pickUpPose.getHeading())
                        .build();


                /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
                park = new Path(new BezierLine(new Point(scorePoseTurned),new Point(parkPose)));
                park.setLinearHeadingInterpolation(scorePoseTurned.getHeading(), parkPose.getHeading());
        }

        /**
         * This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
         * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
         * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on.
         */
        public void autonomousPathUpdate() {
                switch (pathState) {
                        case 0:
                                follower.followPath(scorePreload);
                                setPathState(1);
                                break;
                        case 1:
                /* You could check for
                - Follower State: "if(!follower.isBusy() {}" (Though, I don't recommend this because it might not return due to holdEnd
                - Tim   e: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */
                                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                                if (!follower.isBusy()) {
                                        /* Score Preload */

                                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                                        follower.followPath(goToSamples, true);
                                        setPathState(2);
                                }
                                break;
                        case 2:
                                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                                if (!follower.isBusy()) {
                                        /* Grab Sample */

                                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                                        follower.followPath(lineToSampleOne, false);
                                        setPathState(3);
                                }
                                break;
                        case 3:
                                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                                if (!follower.isBusy()) {
                                        /* Score Sample */

                                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                                        follower.followPath(pushSampleOne, false);
                                        setPathState(4);
                                }
                                break;
                        case 4:
                                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                                if (!follower.isBusy()) {
                                        /* Grab Sample */

                                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                                        follower.followPath(comeBackOne, true);
                                        setPathState(5);
                                }
                                break;
                        case 5:
                                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                                if (!follower.isBusy()) {
                                        /* Score Sample */

                                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                                        follower.followPath(lineToSampleTwo, true);
                                        setPathState(6);
                                }
                                break;
                        case 6:
                                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                                if (!follower.isBusy()) {
                                        /* Grab Sample */

                                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                                        follower.followPath(pushSampleTwo, true);
                                        setPathState(7);
                                }
                                break;
                        case 7:
                                if (!follower.isBusy()) {
                                        /* Grab Sample */

                                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                                        follower.followPath(comeBackTwo, true);
                                        setPathState(8);
                                }
                                break;
                        case 8:
                                if (!follower.isBusy()) {
                                        /* Grab Sample */

                                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                                        follower.followPath(lineToSampleThree, true);
                                        setPathState(9);
                                }
                                break;
                        case 9:
                                if (!follower.isBusy()) {
                                        /* Grab Sample */

                                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                                        follower.followPath(pushSampleThree, true);
                                        setPathState(10);
                                }
                                break;
                        case 10:
                                if (!follower.isBusy()) {
                                        /* Grab Sample */

                                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                                        follower.followPath(pickUpSpecOne, true);
                                        setPathState(11);
                                }
                                break;


                        case 11:
                                if (!follower.isBusy()) {
                                        /* Grab Sample */

                                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                                        follower.followPath(scoreSpec, true);
                                        setPathState(12);
                                }
                                break;
                        case 12:
                                if (!follower.isBusy()) {
                                        /* Grab Sample */

                                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                                        follower.followPath(goBackPickUp, true);
                                        setPathState(13);
                                }
                                break;
                        case 13:
                                if (!follower.isBusy()) {
                                        /* Grab Sample */

                                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                                        follower.followPath(scoreSpec, true);
                                        setPathState(14);
                                }
                                break;
                        case 14:
                                if (!follower.isBusy()) {
                                        /* Grab Sample */

                                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                                        follower.followPath(goBackPickUp, true);
                                        setPathState(15);
                                }
                                break;
                        case 15:
                                if (!follower.isBusy()) {
                                        /* Grab Sample */

                                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                                        follower.followPath(scoreSpec, true);
                                        setPathState(16);
                                }
                                break;
                        case 16:
                                if (!follower.isBusy()) {
                                        /* Grab Sample */

                                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                                        follower.followPath(park, true);
                                        setPathState(17);
                                }
                                break;

                }
        }

        /**
         * These change the states of the paths and actions
         * It will also reset the timers of the individual switches
         **/
        public void setPathState(int pState) {
                pathState = pState;
                pathTimer.resetTimer();
        }

        /**
         * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
         **/
        @Override
        public void loop() {

                // These loop the movements of the robot
                follower.update();
                autonomousPathUpdate();
                // Feedback to Driver Hub
                telemetry.addData("path state", pathState);
                telemetry.addData("x", follower.getPose().getX());
                telemetry.addData("y", follower.getPose().getY());
                telemetry.addData("heading", follower.getPose().getHeading());
                telemetry.update();

        }

        /**
         * This method is called once at the init of the OpMode.
         **/
        @Override
        public void init() {
                pathTimer = new Timer();
                opmodeTimer = new Timer();
                opmodeTimer.resetTimer();
                Constants.setConstants(FConstants.class, LConstants.class);
                follower = new Follower(hardwareMap);
                follower.setStartingPose(startPose);
                buildPaths();
        }

        /**
         * This method is called continuously after Init while waiting for "play".
         **/
        @Override
        public void init_loop() {
        }

        /**
         * This method is called once at the start of the OpMode.
         * It runs all the setup actions, including building paths and starting the path system
         **/
        @Override
        public void start() {
                opmodeTimer.resetTimer();
                setPathState(0);
        }
}