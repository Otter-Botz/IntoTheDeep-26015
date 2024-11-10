package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.common.PID_Arm;


@Config
@TeleOp
public class armTest extends OpMode {
    private PIDController controller;

    public static double p = 0.005, i = 0, d = 0.0001;
    public static double f = 0.01;

    public static int target = 0;

    private final double ticks_in_degrees =  2786.2 / 360;


    public DcMotor armMotor;

    @Override
    public void init() {
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new PIDController(p, i, d);
    }

    @Override
    public void loop() {

        controller.setPID(p, i , d);
        int armPos = armMotor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target/ ticks_in_degrees)) * f;
        double power = pid + ff;
        armMotor.setPower(power);
        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}