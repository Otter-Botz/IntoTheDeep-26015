package org.firstinspires.ftc.teamcode.Autonomous;

import static android.os.SystemClock.sleep;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;


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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.PinPoint.AutoCommonClass;
import org.firstinspires.ftc.teamcode.Autonomous.PinPoint.GoBildaPinpointDriver;
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

@Autonomous(name = "Example Auto Blue", group = "Examples")
public class AutoV2 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    //Claw Positions
    public double ClawOpen = 0.25;
    public double ClawClose = 0;

    //Wrist Positions
    public double WristUp = 0.25;
    public double WristSubmersible = 0.5;
    public double WristDown = 0;
    public double WristMiddle = 0.25;

    //Arm Slider Positions
    public double ArmSliderOut = 1;
    public double ArmSliderIn = 0;
    public double ArmSliderMiddle = 0.5;





    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 111, Math.toRadians(270));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(14, 129, Math.toRadians(315));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
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
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {

                    /* Score Preload */
                    //Need to be tested
                    sliderUpElapsedTime(500);
                    sleep(400);
                    AutoPIDArmmath(1115);
                    sleep(300);
                    clawServo.setPosition(0.25);
                    sleep(500);
                    sliderDownElapsedTime();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    armDownSliderOut();
                    sleep(300);
                    armUpSliderIn();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
                    sliderUpElapsedTime(500);
                    sleep(400);
                    AutoPIDArmmath(1115);
                    sleep(300);
                    clawServo.setPosition(0.25);
                    sleep(500);
                    sliderDownElapsedTime();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    armDownSliderOut();
                    sleep(300);
                    armUpSliderIn();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
                    sliderUpElapsedTime(500);
                    sleep(400);
                    AutoPIDArmmath(1115);
                    sleep(300);
                    clawServo.setPosition(0.25);
                    sleep(500);
                    sliderDownElapsedTime();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup3,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    armDownSliderOut();
                    sleep(300);
                    armUpSliderIn();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
                    sliderUpElapsedTime(500);
                    sleep(400);
                    AutoPIDArmmath(1115);
                    sleep(300);
                    clawServo.setPosition(0.25);
                    sleep(500);
                    sliderDownElapsedTime();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(park,true);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
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

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        initAuto();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }

    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;

    //Servo
    public Servo clawServo;
    public static Servo armSliderServo;
    public Servo wristServo;

    //Slide Motors
    public DcMotor sliderMotorMotor;
    public DcMotor sliderMotor;

    //PID_Arm Motor
    public DcMotor armMotor;

    // Odo Pods and IMU
    GoBildaPinpointDriver odo;
    IMU imu;

    //Elapsed Time
    ElapsedTime runtime = new ElapsedTime();


    //Init Full Auto
    private void initAuto() {
        initImu();
        initDriveMotors();
        initServo();
        initSlider();
        initArmMotor();
        initPinPoint();
    }

    private void initImu() {
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();
    }

    private void initDriveMotors() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void initServo() {
        // Claw/Wrist
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");

        //Arm Slider
        armSliderServo = hardwareMap.get(Servo.class, "servoSlide");
    }

    private void initSlider() {
        sliderMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        sliderMotorMotor = hardwareMap.get(DcMotor.class, "slideMotorMotor");

        sliderMotor.setDirection(DcMotor.Direction.REVERSE);
        sliderMotorMotor.setDirection(DcMotor.Direction.FORWARD);

        sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initArmMotor() {
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        controller = new PIDController(p, i, d);
    }

    private void initPinPoint() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(150, -370); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
    }

    private void initSliderReverse() {
        sliderMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        sliderMotorMotor = hardwareMap.get(DcMotor.class, "slideMotorMotor");

        sliderMotor.setDirection(DcMotor.Direction.FORWARD);
        sliderMotorMotor.setDirection(DcMotor.Direction.REVERSE);

        sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void sliderUpElapsedTime(int position) {
        // Previous Value
        sliderMotor.setTargetPosition(position);
        sliderMotorMotor.setTargetPosition(position);

        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power and let them move to the target
        sliderMotorMotor.setPower(1);  // Adjust power as needed
        sliderMotor.setPower(1);

        // Wait until motors reach their target
        runtime.reset();
        //Tune Runtime Value to move the sliders more or less up

        //Check if this will work
        //Originally was Opmode is active
        while (sliderMotor.isBusy() && runtime.seconds() < 0.7) {
            telemetry.addData("Motor Left Current Inside", sliderMotor.getCurrentPosition());
            telemetry.addData("Motor Right Current Inside", sliderMotorMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop the motors once target is reached
        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private static PIDController controller;
    //p = 0.005 i = 0 d = 0.0001 f=0.01
    private static final double p = 0.005;
    private static final double i = 0;
    private static final double d = 0.0001;
    private static final double f = 0.01;
    private final double target = 100;
    private final double ticks_in_degrees = 2786.2 / 360;

    //Math

    public void AutoPIDArmmath(double target) {
        controller.setPID(p, i, d);
        int armPos = armMotor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
        double power = pid + ff;
        armMotor.setPower(power);
        // telemetry.addData("pos", slidePos);
        // telemetry.addData("target", target);
        // telemetry.update();
    }


    private ElapsedTime timer = new ElapsedTime();

    public void sliderDownElapsedTime() {
        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sliderMotor.setPower(-1);
        sliderMotorMotor.setPower(-1);

        timer.reset();
        // In your loop, stop the motor after a certain time
        while (timer.seconds() < 0.1) {
            telemetry.addData("Motor Left Current Inside", sliderMotor.getCurrentPosition());
            telemetry.addData("Motor Right Current Inside", sliderMotorMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop the sliders once target positions are reached
        sliderMotor.setPower(0);
        sliderMotorMotor.setPower(0);
    }

    public void armDown() {
        runtime.reset();
        // Run tasks for the entire autonomous period
        while (runtime.seconds() < 1) {
            AutoPIDArmmath(0);
        }
    }

    public void armDownSliderOut() {
        sleep(200);
        runtime.reset();
        // Run tasks for the entire autonomous period
        while (runtime.seconds() < 1) {
            AutoPIDArmmath(200);
        }
        armSliderServo.setPosition(ArmSliderOut);
        sleep(300);
        clawServo.setPosition(ClawOpen);
        sleep(600);
        clawServo.setPosition(ClawClose);
        sleep(200);
        wristServo.setPosition(WristUp);
    }

    public void armUpSliderIn() {
        runtime.reset();
        // Run tasks for the entire autonomous period
        while (runtime.seconds() < 1) {
            AutoPIDArmmath(950);
        }
        armSliderServo.setPosition(ArmSliderIn);
    }



}
