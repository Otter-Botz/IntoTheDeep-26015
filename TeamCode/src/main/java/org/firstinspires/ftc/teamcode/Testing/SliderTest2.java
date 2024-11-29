package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.PID_Slider;

@TeleOp(name = "SliderTest2", group = "otterbotz")
public class SliderTest2 extends LinearOpMode {

    public DcMotor slideMotorMotor;
    public DcMotor slideMotor;
    //0.00042224 down
    //0.0004223 Up
    //This Hold speed is going to drive me CRAZY
    //0.00042219999999
    public static final double SLIDER_HOLD_SPEED = 0.00042229;

    double kP = 0.000001;
    double kI = 0.001;
    double kD = 0.1;

    int targetPosition = 400;
    int lastError = 0;
    int integral = 0;
    int error = 0;
    int prevPosition = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        slideMotorMotor = hardwareMap.get(DcMotor.class, "slideMotorMotor");
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideMotor.setDirection(DcMotor.Direction.REVERSE);
        slideMotorMotor.setDirection(DcMotor.Direction.FORWARD);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while(opModeInInit()){
            telemetry.addData("Init pos motor1", slideMotor.getCurrentPosition());
            telemetry.addData("Init pos motor2", slideMotorMotor.getCurrentPosition());
            telemetry.addData("Init TargetPositionMotor1", slideMotor.getTargetPosition());
            telemetry.addData("Init TargetPositionMotor2", slideMotorMotor.getTargetPosition());
            telemetry.update();
        }

        waitForStart();

//        slideMotor.setTargetPosition(targetPosition);
//        slideMotorMotor.setTargetPosition(targetPosition);

        slideMotor.setPower(0.2);
        slideMotorMotor.setPower(0.2);
//        PID_Slider pidSlideMotor = new PID_Slider(slideMotor, 400);
//        PID_Slider pidSlideMotorMotor = new PID_Slider(slideMotorMotor, 400);

        while (opModeIsActive()) {

            int currentPosition = Math.abs(slideMotor.getCurrentPosition());

            error = targetPosition - currentPosition;

            integral += error;

            int derivative = error - lastError;

            double pidOutput = kP * error + kI * integral + kD * derivative;

            slideMotor.setPower(Range.clip(pidOutput, -1.0, 1.0));
            slideMotorMotor.setPower(Range.clip(pidOutput, -1.0, 1.0));

            lastError = error;

            telemetry.addData("pos motor1", slideMotor.getCurrentPosition());
            telemetry.addData("pos motor2", slideMotorMotor.getCurrentPosition());
            telemetry.addData("TargetPositionMotor1", slideMotor.getTargetPosition());
            telemetry.addData("TargetPositionMotor2", slideMotorMotor.getTargetPosition());
            telemetry.addData("Error", error);
            telemetry.addData("PID Output", pidOutput);
            telemetry.update();



//            if (gamepad1.x) {
//                slideMotor.setTargetPosition(1400);
//                slideMotorMotor.setTargetPosition(1400);
//                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideMotor.setPower(0.3);
//                slideMotorMotor.setPower(0.3);
//                sleep(500);
//
//
//
////                while (opModeIsActive() && (slideMotor.isBusy() || slideMotorMotor.isBusy())) {
////                    telemetry.addData("pos motor1", slideMotor.getCurrentPosition());
////                    telemetry.addData("pos motor2", slideMotorMotor.getCurrentPosition());
////                    telemetry.update();
////                }
//            }


//            pidSlideMotor.math();
//            pidSlideMotorMotor.math();

//            slideMotor.setPower(SLIDER_HOLD_SPEED);
//            slideMotorMotor.setPower(SLIDER_HOLD_SPEED);

            // Set motors back to normal encoder mode

        }
    }
}
