package org.firstinspires.ftc.teamcode.Autonomous.Common;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.text.method.Touch;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class PID_Arm {
    // auto arm stuff
        private PIDController controller;
        public DcMotor armMotor;
        public TouchSensor touchSensor;
    public static double p = 0.005, i = 0, d = 0.0001;
    public static double f = 0.01;
    double ARM_UP = 820;
    double ARM_DOWN = 110;
    double ARM_BACK = 1700;
    double ARM_START = 150;
    double ARM_MIDDLE = 540;
    public double target = ARM_START;

    private final double ticks_in_degrees = 2786.2 / 360;



    public PID_Arm(HardwareMap hardwareMap) {
            armMotor = hardwareMap.get(DcMotor.class, "armMotor");
            touchSensor = hardwareMap.get(TouchSensor.class, "sensorTouch");
            controller = new PIDController(p, i, d);
    }
    public class autoEnd implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            target = 0;
            return false;
        }

    }
    public Action autoEnd(){
        return new autoEnd();
    }


        //arm  up stuff
        public class up implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                target = ARM_UP;

                return false;
            }

        }
        public Action armUp(){
            return new up();
        }
        public int pos(){
        return armMotor.getCurrentPosition();
        }

        // arm all the way down
        public class down implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                target = ARM_DOWN;
                return false;
            }

        }
        public Action armDown(){
            return new down();
        }

        public class middle implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            target = ARM_MIDDLE;
            return false;
        }

        }
        public Action middle(){
        return new middle();
    }


    public class backDown implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                target = ARM_BACK;
                return false;
            }

        }
        public Action backDown(){
            return new backDown();
        }






    //math for PID
        public class math implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                controller.setPID(p, i , d);
                int armPos = armMotor.getCurrentPosition();
                double pid = controller.calculate(armPos, target);
                double ff = Math.cos(Math.toRadians(target/ ticks_in_degrees)) * f;
                double power = pid + ff;
                armMotor.setPower(power);
                return true;
            }
        }
        public Action mathRun() {
            return new math();
        }

      public class touch implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return false;
        }

      }

        public Action touchreset() {
        return new touch();
        }


    }





