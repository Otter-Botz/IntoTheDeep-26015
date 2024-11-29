package org.firstinspires.ftc.teamcode.Autonomous.PinPoint;



import static android.os.SystemClock.sleep;

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
    public void clawOpen(){
        clawServo.setPosition(0);
    }
    public void clawClose(){
        clawServo.setPosition(0.3);
    }


    //wrist positions
    public void wristUp(){
        wristServo.setPosition(0.6);
    }
//    public void wristDown(){
//        wristDown();
//    }




    //only spec auto arm movements
    public void backGrab() {
        armMotor.setTargetPosition(200);
    }
    public void belowRung(){
        armMotor.setTargetPosition(1500);
    }
    public void aboveRung(){
        armMotor.setTargetPosition(1200);
    }
    //arm movement used for both autos
    public void downGrab(){
        armMotor.setTargetPosition(221);
    }
    public void armUp(){
        armMotor.setTargetPosition(1115);
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
        armUp();
        //Wrist
        wristUp();
        sleep(1000);
        clawOpen();

    }

    public void scoreSpecimen() {

    }

    //slider movement used for high basket auto
    public void highBasket(){
        sliderMotor.setTargetPosition(900);
        sliderMotorMotor.setTargetPosition(900);
    }

}
