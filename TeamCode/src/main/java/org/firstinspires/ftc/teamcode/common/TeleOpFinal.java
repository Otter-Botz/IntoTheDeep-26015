package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "OTTERRRRR")
public class TeleOpFinal extends LinearOpMode {
    claw claw = new claw();
    ArmSlider ArmSlider = new ArmSlider();
    PID_Arm PID_Arm = new PID_Arm();
    vroomVroom vroom = new vroomVroom();
    PID_Slider PID_Slider = new PID_Slider();


    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
       claw.init(hardwareMap);
       ArmSlider.init(hardwareMap);
        PID_Arm.init(hardwareMap);
        vroom.init(hardwareMap);
        PID_Slider.init(hardwareMap);

        vroom.vrooooooom(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_trigger );
        PID_Arm.math();

        if (gamepad1.options) {
            vroom.resetYaw();
        }
        else if (gamepad1.a){
            claw.set1();
        }
        else if (gamepad1.a){
            claw.set2();
        }
        else if (gamepad1.x){
            PID_Arm.up();
        }
        else if (gamepad1.y){
            PID_Arm.down();
        }
        else if (gamepad1.left_stick_x<1) {
            ArmSlider.set1();
        }
        //bogos binted
        //hello








    }
}






