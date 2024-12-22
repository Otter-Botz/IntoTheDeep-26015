package org.firstinspires.ftc.teamcode.Autonomous.PinPoint;

public interface autoCommonInterface {
    //Init Full Auto
    void initAuto();

    void set(double position);

    //Adjust Positions
    void MainSliderPositionChange();

    void SliderMath();

    void sliderUp();

    void scoreHighBasket();

    //Math
    void AutoPIDArmmath(double target);

    void armDown();

    void lowbasketslider();

    void lowbasketsliderwithoutwrist();

    void driveToPos(double targetX, double targetY);

    //Turn
    void gyroTurnToAngle(double turnAngle);
}
