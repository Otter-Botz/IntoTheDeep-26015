package org.firstinspires.ftc.teamcode.Autonomous.Common;

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

    public void init(HardwareMap hwMap) {
        sliderMotor = hwMap.get(DcMotor.class, "slideMotor");
        sliderMotorMotor = hwMap.get(DcMotor.class, "slideMotorMotor");
        sliderMotorMotor.setDirection(DcMotor.Direction.REVERSE);

    }

    public void setSliderIdlePosition() {
        sliderMotor.setPower(IDLE_SPEED);
        sliderMotorMotor.setPower(IDLE_SPEED);
        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void  LowBaskets (){
        sliderMotor.setTargetPosition(1350);
        sliderMotorMotor.setTargetPosition(1350);
        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotor.setPower(SLIDER_UP_SPEED);
        sliderMotorMotor.setPower(SLIDER_UP_SPEED);
        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setSliderIdlePosition();
    }
    public void  HighRung (){
        sliderMotor.setTargetPosition(1750);
        sliderMotorMotor.setTargetPosition(1750);
        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotor.setPower(SLIDER_UP_SPEED);
        sliderMotorMotor.setPower(SLIDER_UP_SPEED);
        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setSliderIdlePosition();
    }
    public void  LowRung (){
        sliderMotor.setTargetPosition(1350);
        sliderMotorMotor.setTargetPosition(1350);
        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotor.setPower(SLIDER_UP_SPEED);
        sliderMotorMotor.setPower(SLIDER_UP_SPEED);
        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setSliderIdlePosition();
    }
    public void  HighBaskets (){
        sliderMotor.setTargetPosition(1750);
        sliderMotorMotor.setTargetPosition(1750);
        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotor.setPower(SLIDER_UP_SPEED);
        sliderMotorMotor.setPower(SLIDER_UP_SPEED);
        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setSliderIdlePosition();
    }

}
