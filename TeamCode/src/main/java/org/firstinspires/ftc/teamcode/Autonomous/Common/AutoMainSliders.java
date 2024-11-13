package org.firstinspires.ftc.teamcode.Autonomous.Common;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutoMainSliders {
    // Slider
    public static final double IDLE_SPEED = 0.0;

    public DcMotor sliderMotor;
    public DcMotor sliderMotorMotor;
    public static final double SLIDER_UP_SPEED = 1.0;
    public static final double SLIDER_DOWN_SPEED = 0.5;
    public static final double SLIDER_HOLD_SPEED = 0.001;
    public static final double SLIDER_HIGH_RUNG = 450;
    public static final double SLIDER_LOW_RUNG = 250;
    public static final double HIGH_BASKET_MAIN_SLIDER = 300;
    public static final double LOW_BASKET_MAIN_SLIDER = 150;

    public AutoMainSliders(HardwareMap hardwareMap) {
    }



    public void init(HardwareMap hwMap) {
        sliderMotor = hwMap.get(DcMotor.class, "slideMotor");
        sliderMotorMotor = hwMap.get(DcMotor.class, "slideMotorMotor");
        sliderMotorMotor.setDirection(DcMotor.Direction.REVERSE);

    }

    public class HighRung implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sliderMotor.setTargetPosition(250);
            sliderMotorMotor.setTargetPosition(250);
            sliderMotor.setPower(SLIDER_UP_SPEED);
            sliderMotorMotor.setPower(SLIDER_UP_SPEED);
            sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return false;
        }
    }
    public Action HighRung() {
        return new AutoMainSliders.HighRung();
    }

    public class LowRung implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            sliderMotor.setTargetPosition(100);
            sliderMotorMotor.setTargetPosition(100);
            sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sliderMotor.setPower(SLIDER_UP_SPEED);
            sliderMotorMotor.setPower(SLIDER_UP_SPEED);
            sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return false;
        }
    }
    public Action LowRung() {
        return new LowRung();
    }

    public class HighBasket implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            sliderMotor.setTargetPosition(400);
            sliderMotorMotor.setTargetPosition(400);
            sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sliderMotor.setPower(SLIDER_UP_SPEED);
            sliderMotorMotor.setPower(SLIDER_UP_SPEED);
            sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return false;
        }
    }
    public Action HighBasket() {
        return new HighBasket();
    }

    public class SliderIdlePosition implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            sliderMotor.setPower(IDLE_SPEED);
            sliderMotorMotor.setPower(IDLE_SPEED);
            sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return false;
        }
    }
    public Action SliderIdlePosition() {
        return new SliderIdlePosition();
    }
    public class LowBasket implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            sliderMotor.setTargetPosition(200);
            sliderMotorMotor.setTargetPosition(200);
            sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sliderMotor.setPower(SLIDER_UP_SPEED);
            sliderMotorMotor.setPower(SLIDER_UP_SPEED);
            return false;
        }
    }
    public Action Lowbasket() {
        return new LowBasket();
    }

    public class SliderDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sliderMotor.setTargetPosition(0);
            sliderMotorMotor.setTargetPosition(0);
            sliderMotor.setPower(SLIDER_UP_SPEED);
            sliderMotorMotor.setPower(SLIDER_UP_SPEED);
            sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return false;
        }
    }
    public Action SliderDown() {
        return new SliderDown();
    }



//    public void setSliderIdlePosition() {
//        sliderMotor.setPower(IDLE_SPEED);
//        sliderMotorMotor.setPower(IDLE_SPEED);
//        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//      public void  HighRung (){
//        sliderMotor.setTargetPosition((int) SLIDER_HIGH_RUNG);
//        sliderMotorMotor.setTargetPosition((int) SLIDER_HIGH_RUNG);
//        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        sliderMotor.setPower(SLIDER_UP_SPEED);
//        sliderMotorMotor.setPower(SLIDER_UP_SPEED);
//        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        setSliderIdlePosition();
//    }
//    public void  LowRung (){
//        sliderMotor.setTargetPosition(1350);
//        sliderMotorMotor.setTargetPosition(1350);
//        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        sliderMotor.setPower(SLIDER_UP_SPEED);
//        sliderMotorMotor.setPower(SLIDER_UP_SPEED);
//        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        setSliderIdlePosition();
//    }
//    public void  HighBaskets (){
//        sliderMotor.setTargetPosition(1750);
//        sliderMotorMotor.setTargetPosition(1750);
//        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        sliderMotor.setPower(SLIDER_UP_SPEED);
//        sliderMotorMotor.setPower(SLIDER_UP_SPEED);
//        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        setSliderIdlePosition();
//    }

}
