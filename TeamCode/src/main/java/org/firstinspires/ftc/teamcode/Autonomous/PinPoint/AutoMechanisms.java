package org.firstinspires.ftc.teamcode.Autonomous.PinPoint;



import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.ArmSlider;
import org.firstinspires.ftc.teamcode.common.PID_Arm;
import org.firstinspires.ftc.teamcode.common.Slider;
import org.firstinspires.ftc.teamcode.common.claw;
import org.firstinspires.ftc.teamcode.common.vroomVroom;
import org.firstinspires.ftc.teamcode.common.wrist;

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
  public void wristDown(){
       wristDown();
   }

//pid math
    public void pidMath(){
        
    }


    //only spec auto arm movements
    public void backGrab() {
        armMotor.setTargetPosition(200);
    }
    public void belowRung(){
        armMotor.setTargetPosition(1500);
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
