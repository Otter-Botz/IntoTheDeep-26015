package org.firstinspires.ftc.teamcode.common;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "OTTERRRRR")
public class TeleOpFinal extends LinearOpMode {
    claw claw = new claw();
    ArmSlider ArmSlider = new ArmSlider();
    PID_Arm PID_Arm = new PID_Arm();
    vroomVroom vroom = new vroomVroom();
    Slider Slider = new Slider();
    wrist wrist = new wrist();



    @Override
    public void runOpMode() throws InterruptedException {

        claw.init(hardwareMap);
        ArmSlider.init(hardwareMap);
        PID_Arm.init(hardwareMap);
        vroom.init(hardwareMap);
        Slider.init(hardwareMap);
        wrist.init(hardwareMap);
        boolean clawPressed = false;



        waitForStart();

        PID_Arm.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PID_Arm.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            telemetry.addData("pos", PID_Arm.armMotor.getCurrentPosition());
            telemetry.addData("servopos", wrist.wristServo.getPosition());
            telemetry.update();


            vroom.vrooooooom(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_trigger);
            PID_Arm.math();



            //Rumble and Reset Yaw
            if (gamepad1.options) {
                vroom.resetYaw();
            }

            if (gamepad1.right_bumper) {
                gamepad2.rumble(1000);
            }

            clawPressed = clawPressed && gamepad1.b;

            if(gamepad1.b && !clawPressed){
                toggleClaw();
                clawPressed = true;
            }
            //claw code

            // preset

            if (gamepad2.b) {
                highbasket();
            } else if (gamepad2.a) {
                submersible();
            }

            if (gamepad2.x) {
                wrist.set(0);
            }



            Slider.sliderMotor.setPower(-gamepad2.right_stick_y);
            Slider.sliderMotorMotor.setPower(-gamepad2.right_stick_y);
            ArmSlider.armSliderServo.setPower(gamepad2.left_stick_y);


            if (gamepad2.left_bumper) {
                claw.clawServo.setPosition(0);
            } else if (gamepad2.right_bumper) {
                wrist.wristServo.setPosition(0);
            }



            // fine control (ryan was here)

            if (gamepad2.dpad_up) {
                double value;
                value = PID_Arm.target + 3;
                PID_Arm.target = value;
            } else if (gamepad2.dpad_down){
                double value1;
                value1 = PID_Arm.target - 3;
                PID_Arm.target = value1;
            }


            //telemetry.addData("pos", PID_Arm.armMotor.getCurrentPosition());
            //telemetry.update();


        }


    }

    private  void toggleArm() {
        if(PID_Arm.getPosition() == PID_Arm.Up){
            PID_Arm.set(PID_Arm.Down);
        }
        else if(PID_Arm.getPosition() != PID_Arm.Up){
            PID_Arm.set(PID_Arm.Up);
        }
    }

    private  void toggleClaw(){
        if(claw.getPosition() == claw.close){
            claw.set(claw.open);
        } else if(claw.getPosition() != claw.close){
            claw.set(claw.close);
        }
    }
    private  void toggleWrist(){
        if(wrist.getPosition() == wrist.up){
            wrist.set(wrist.down);
        } else if(wrist.getPosition() != wrist.up){
            wrist.set(wrist.up);
        }
    }

    public void highbasket() {
        PID_Arm.up();
        wrist.set(wrist.down);
    }


    public void submersible() {
        PID_Arm.down();
        wrist.set(wrist.up);
    }

}








