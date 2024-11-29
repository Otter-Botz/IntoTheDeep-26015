package org.firstinspires.ftc.teamcode.Autonomous.PinPoint.Mech;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class armSlide {

    boolean rotation;
    Servo armServo;
    public double angleWrap(double radians) {
        radians = Math.toRadians(armServo.getPosition());

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        return radians;
    }

    public double runToPos(double angle) {
        Math.toDegrees(angleWrap(Math.toRadians(angle)));

        if (angle > 355) {
            rotation = true;
        }

        armServo.setPosition(angle);


        return angle;

    }









}
