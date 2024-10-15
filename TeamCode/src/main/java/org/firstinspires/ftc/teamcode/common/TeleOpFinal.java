package org.firstinspires.ftc.teamcode.common;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleOpFinal extends LinearOpMode {
    claw claw = new claw();
    ArmSlider armSlider = new ArmSlider();
    PID_Arm PID_Arm = new PID_Arm();
    vroomVroom vroom = new vroomVroom();





    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        claw.init(hardwareMap);
        armSlider.init(hardwareMap);
        PID_Arm.init(hardwareMap);
        vroom.init(hardwareMap);

        vroom.math(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_trigger );
        PID_Arm.math();

        if (gamepad1.options) {
            vroom.resetYaw();
        }









    }
}






