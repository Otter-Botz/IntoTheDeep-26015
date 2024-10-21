package org.firstinspires.ftc.teamcode.common;
/*
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class PID_Arm extends OpMode {

    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degrees = 1425.1/360;
    private DcMotor armMotor;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void loop() {

        controller.setPID(p, i , d);
        int slidePos = armMotor.getCurrentPosition();
        double pid = controller.calculate(slidePos, target);
        double ff = Math.cos(Math.toRadians(target/ ticks_in_degrees)) * f;
        double power = pid + ff;
        armMotor.setPower(power);
        telemetry.addData("pos", slidePos);
        telemetry.addData("target", target);
        telemetry.update();

    }


}
*/




import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config

public class PID_Arm  {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degrees = 700/180;

    private DcMotor armMotor;





    public void up() {
        target = 600;
    }


    public void down() {
        target = 300;
    }


    public void math() {


        controller.setPID(p, i , d);
        int slidePos = armMotor.getCurrentPosition();
        double pid = controller.calculate(slidePos, target);
        double ff = Math.cos(Math.toRadians(target/ ticks_in_degrees)) * f;
        double power = pid + ff;
        armMotor.setPower(power);
        // add tele back into main class
       // telemetry.addData("pos", slidePos);
       // telemetry.addData("target", target);
       // telemetry.update();
    }



    public void init(HardwareMap hwMap) {
        armMotor = hwMap.get(DcMotor.class, "armMotor");
        controller = new PIDController(p, i, d);
    }
    // telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

}
