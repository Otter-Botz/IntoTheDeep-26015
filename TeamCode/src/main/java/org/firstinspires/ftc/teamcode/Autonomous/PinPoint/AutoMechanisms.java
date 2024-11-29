package org.firstinspires.ftc.teamcode.Autonomous.PinPoint;



import static android.os.SystemClock.sleep;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.PID_Arm;

public class AutoMechanisms {

    // Drive Motors
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    // Sliders
    private DcMotor sliderMotor;
    private DcMotor sliderMotorMotor;

    //PID_Arm
    private DcMotor armMotor;

    // Claw/Wrist
    private Servo clawServo;
    private Servo wristServo;

    //PID_Arm
    public final double PID_ArmUp = 1115;
    public final double PID_ArmDown = 0;
    public final double PID_ArmMiddle = 557.5;



        //claw positions
        public void clawOpen() {
            clawServo.setPosition(0);
        }

        public void clawClose() {
            clawServo.setPosition(0.3);
        }




        //wrist positions
        public void wristUp() {
            wristServo.setPosition(0.6);
        }

        public void wristDown() {
            wristDown();
        }




    //only spec auto arm movements
    public void backGrab() {
        armMotor.setTargetPosition(200);
    }

    public void belowRung() {
        armMotor.setTargetPosition(1500);
    }

    public void aboveRung() {
        armMotor.setTargetPosition(1200);
    }

    //arm movement used for both autos
    public void downGrab() {
        armMotor.setTargetPosition(221);
    }

    public void armUp() {
        armMotor.setTargetPosition(1115);
    }

    private static PIDController controller;
    //p = 0.005 i = 0 d = 0.0001 f=0.01
    public static double p = 0.005, i = 0, d = 0.0001;
    public static double f = 0.01;

    public double target = 100;

    private final double ticks_in_degrees = 2786.2/ 360;
    //2786.2

    public void  math() {
        controller.setPID(p, i, d);
        int armPos = armMotor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
        double power = pid + ff;
        armMotor.setPower(power);
        // add tele back into main class
        // telemetry.addData("pos", slidePos);
        // telemetry.addData("target", target);
        // telemetry.update();
    }


//    public void pickupsample() {
//
//        wristDown();
//        clawOpen();
//        sleep(500);
//        clawClose();
//    }

    public void scoreSample() {

        //Main Sliders

        //Arm
       // armUp();
        //Wrist
       // wristUp();
        sleep(1000);
        //clawOpen();

    }

    public void scoreSpecimen() {

    }


        //slider movement used for high basket auto
        public void highBasket() {
            sliderMotor.setTargetPosition(900);
            sliderMotorMotor.setTargetPosition(900);
        }
    }

