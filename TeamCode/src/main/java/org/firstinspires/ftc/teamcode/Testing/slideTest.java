package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.PID_Arm;
import org.firstinspires.ftc.teamcode.common.claw;
import org.firstinspires.ftc.teamcode.common.wrist;

@TeleOp(name = "slideTest", group = "otterbotz")
public class slideTest extends LinearOpMode {

    public DcMotor slideMotorMotor;
    public DcMotor slideMotor;
    public DcMotor armMotor;
    //0.00042224 down
    //0.0004223 Up
    //This Hold speed is going to drive me CRAZY
    //0.00042219999999
    public static final double SLIDER_HOLD_SPEED = 0.00042229;

    PID_Arm PID_Arm = new PID_Arm();
    wrist wrist = new wrist();
    claw claw = new claw();

    @Override
    public void runOpMode() throws InterruptedException {

        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        slideMotorMotor = hardwareMap.get(DcMotor.class, "slideMotorMotor");
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideMotor.setDirection(DcMotor.Direction.REVERSE);
        slideMotorMotor.setDirection(DcMotor.Direction.FORWARD);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        PID_Arm.init(hardwareMap);
        wrist.init(hardwareMap);
        claw.init(hardwareMap);

        claw.AutoClose();
        wrist.set(wrist.up);

        while (opModeInInit()) {
            telemetry.addData("pos motor1", slideMotor.getCurrentPosition());
            telemetry.addData("pos motor2", slideMotorMotor.getCurrentPosition());

            telemetry.addData("pos motor1 target", slideMotor.getTargetPosition());
            telemetry.addData("pos motor2 target", slideMotorMotor.getTargetPosition());

            //telemetry.addData("PID_Arm", PID_Arm.getPosition());
            telemetry.addData("PID_ArmCurrent", org.firstinspires.ftc.teamcode.common.PID_Arm.armMotor.getCurrentPosition());
            telemetry.addData("servoPos", wrist.wristServo.getPosition());
            telemetry.addData("clawPos", claw.clawServo.getPosition());


            telemetry.update();
        }
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {

            //claw.AutoClose();
            //PID_Arm.math();

            telemetry.addData("pos motor1", slideMotor.getCurrentPosition());
            telemetry.addData("pos motor2", slideMotorMotor.getCurrentPosition());
            telemetry.update();

            if (gamepad1.x) {
                claw.AutoOpen();

            }

            if (gamepad1.a){
                claw.AutoClose();
            }

//
//        claw.AutoClose();
//        //armMotor.setTargetPosition(1115);
//      //  sleep(5000);
////        PID_Arm.AutoUp();
////        PID_Arm.math();
//
//        slideMotor.setTargetPosition(1900);
//        slideMotorMotor.setTargetPosition(1900);
//        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        slideMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        slideMotor.setPower(0.1);
//        slideMotorMotor.setPower(0.1);
//        sleep(5000);
//
//        slideMotor.setPower(SLIDER_HOLD_SPEED);
//        slideMotorMotor.setPower(SLIDER_HOLD_SPEED);
//
//        wrist.set(wrist.down);
//        sleep(5000);
//        claw.AutoOpen();


//        PID_Slider pidSlideMotor = new PID_Slider(slideMotor, 400);
//        PID_Slider pidSlideMotorMotor = new PID_Slider(slideMotorMotor, 400);

//        while (opModeIsActive()) {
//
//            telemetry.addData("pos motor1", slideMotor.getCurrentPosition());
//            telemetry.addData("pos motor2", slideMotorMotor.getCurrentPosition());
//
//            telemetry.addData("pos motor1 target", slideMotor.getTargetPosition());
//            telemetry.addData("pos motor2 target", slideMotorMotor.getTargetPosition());
//
//            telemetry.addData("PID_Arm", PID_Arm.getPosition());
//            telemetry.addData("PID_ArmCurrent", org.firstinspires.ftc.teamcode.common.PID_Arm.armMotor.getCurrentPosition());
//            telemetry.addData("servoPos", wrist.wristServo.getPosition());
//            telemetry.addData("clawPos", claw.clawServo.getPosition());
//            telemetry.update();
//
//            if (gamepad1.x) {
//                claw.AutoClose();
//                slideMotor.setTargetPosition(1900);
//                slideMotorMotor.setTargetPosition(1900);
//                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideMotor.setPower(0.5);
//                slideMotorMotor.setPower(0.5);
//                sleep(1000);
//
//                slideMotor.setPower(SLIDER_HOLD_SPEED);
//                slideMotorMotor.setPower(SLIDER_HOLD_SPEED);
//
//                PID_Arm.AutoUp();
//
//                wrist.set(wrist.down);
//
//                if(wrist.getPosition() == wrist.down){
//                claw.AutoOpen();
//            }
//
//            }
//            PID_Arm.math();
//
////            slideMotor.setPower(SLIDER_HOLD_SPEED);
////            slideMotorMotor.setPower(SLIDER_HOLD_SPEED);
////
////
//////            if(slideMotor.get)
////            if(wrist.getPosition() == wrist.down){
////                claw.AutoOpen();
////            }
////
////
//////            pidSlideMotor.math();
//////            pidSlideMotorMotor.math();
////
////
////            // Set motors back to normal encoder mode
////            slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////            slideMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
            }
        }
    }