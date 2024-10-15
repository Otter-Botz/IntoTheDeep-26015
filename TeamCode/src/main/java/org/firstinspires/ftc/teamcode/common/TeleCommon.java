package org.firstinspires.ftc.teamcode.common;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class TeleCommon extends LinearOpMode {
    claw claw = new claw();
    ArmSlider armSlider = new ArmSlider();
    PID_Arm PID_Arm = new PID_Arm();

    vroomVroom vroom = new vroomVroom();





    @Override
    public void runOpMode() throws InterruptedException {
       claw.init(hardwareMap);
       armSlider.init(hardwareMap);
        PID_Arm.init(hardwareMap);
        vroom.init(hardwareMap);
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double power = gamepad1.right_trigger;


    }
}






