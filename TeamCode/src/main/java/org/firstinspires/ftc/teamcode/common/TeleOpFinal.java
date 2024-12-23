package org.firstinspires.ftc.teamcode.common;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

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
    private LinearOpMode linearOpMode;
    double subArmPos = 300;

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

            if (gamepad1.right_stick_button) {
                specimen();
            }




//            public double open = 0.55;
//            public double close = 0.37 ;

            if (gamepad1.a) {
                claw.set(claw.open);
            }
            if(gamepad1.b){
                claw.set(claw.close);
            }
            if (gamepad2.right_bumper) {
                claw.set(claw.open);
            }

            if (gamepad1.left_bumper) {
                ArmSlider.set(ArmSlider.out);
                PID_Arm.target = subArmPos;
            }
            else if (gamepad1.right_bumper) {
                ArmSlider.set(ArmSlider.in);
                PID_Arm.target = subArmPos;
            }
            else if (gamepad1.dpad_right) {
                PID_Arm.target = subArmPos;
                ArmSlider.set(ArmSlider.middle);

            }
            else if (gamepad1.left_stick_button) {
                PID_Arm.target = 200;
            }


            //claw code

            // preset
            if (gamepad2.x) {
                specimen();
            }

            if (gamepad2.b) {
                highbasket();
            } else if (gamepad2.a) {
                submersible();
            }



            Slider.sliderMotor.setPower(gamepad2.right_stick_y);
            Slider.sliderMotorMotor.setPower(gamepad2.right_stick_y);


            if (gamepad2.left_bumper) {
                claw.clawServo.setPosition(0);
            } else if (gamepad2.left_bumper) {
                wrist.wristServo.setPosition(0);
            }

            // Arm reset button
            if (gamepad2.options) {
                PID_Arm.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                PID_Arm.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }



            // fine control (ryan was here)

            if (gamepad2.right_trigger == 1) {
                double value;
                value = PID_Arm.target + 4;
                PID_Arm.target = value;
            } else if (gamepad2.left_trigger == 1){
                double value1;
                value1 = PID_Arm.target - 4;
                PID_Arm.target = value1;
            }


            if (gamepad1.left_trigger > 0){
                double value2;
                value2 = PID_Arm.target - 4*gamepad1.left_trigger;
                PID_Arm.target = value2;
            }





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
        wrist.set(wrist.up);
    }

    public void specimen() {
        PID_Arm.specimen();
        wrist.set(wrist.up);
    }

    public void submersible() {
        PID_Arm.down();
        wrist.set(wrist.down);
    }

    public void highbasketslider() {
        Slider.sliderMotor.setTargetPosition(900);
        Slider.sliderMotorMotor.setTargetPosition(900);
        Slider.sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider.sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider.sliderMotor.setPower(0.5);
        Slider.sliderMotorMotor.setPower(0.5);
        PID_Arm.up();
        wrist.set(wrist.down);
    }

    public void sliderwork() {
        Slider.sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slider.sliderMotorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slider.sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider.sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider.sliderMotor.setTargetPosition(400);
        Slider.sliderMotorMotor.setTargetPosition(400);
        Slider.sliderMotor.setPower(0.5);
        Slider.sliderMotorMotor.setPower(0.5);
    }

    public void highBaskets(){

        Slider.sliderMotor.setTargetPosition(900);
        Slider.sliderMotorMotor.setTargetPosition(900);
        Slider.sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider.sliderMotorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider.sliderMotor.setPower(0.5);
        Slider.sliderMotorMotor.setPower(0.5);
        Slider.sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slider.sliderMotorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Preset Please Work
        PID_Arm.up();
        wrist.set(wrist.down);

    }





}













