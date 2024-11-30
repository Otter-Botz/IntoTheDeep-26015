package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;




/// DONT DELETE COMMENT
/*
@Config
@TeleOp
public class PID_Arm extends OpMode {

    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degrees = 2786.2/360;
    public DcMotor armMotor;

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


//import org.firstinspires.ftc.teamcode.common.interfaces.PIDArm;


@Config

public class PID_Arm {
    private static PIDController controller;
    //p = 0.005 i = 0 d = 0.0001 f=0.01
    public static double p = 0.005, i = 0, d = 0.0001;
    public static double f = 0.01;

    public double target = 100;

    private final double ticks_in_degrees = 2786.2/ 360;
    //2786.2

    public final double Up = 1115;
    public final double Down = 0;
    //to be tested
    public final double Auto_grab = 200;
    public final double Auto_belowRung = 1500;
    public final double Auto_aboveRung = 1200;




    public static DcMotor armMotor;

    public PID_Arm() {

    }

    public void AutoUp() {
        target = -1195;
    }




    public void specimen() {
        target = 1350;
    }

    public void up() {
        target = 1115;
    }

    public void idle() {
         target = 300;
    }

    public void armRespond(double value) {
        value = value * 1;
        target = target - value;
    }

    public void down() {
        target = 221;
    }


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

    public void  math(double target) {
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


//    public double getPosition() {
//        return armMotor.getCurrentPosition();
//    }


    public void init(HardwareMap hwMap) {
        armMotor = hwMap.get(DcMotor.class, "armMotor");

        controller  = new PIDController(p,i,d);
    }
}










