package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Config
@TeleOp
public class PID_Slider extends OpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degrees = 70/180;

    private DcMotor LinearSlide1;


    @Override
    public void init() {
        controller = new PIDController(p,i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LinearSlide1 = hardwareMap.get(DcMotor.class, "LinearSlide1");
      

    }

    @Override
    public void loop() {
        controller.setPID(p, i , d);
        int slidePos = LinearSlide1.getCurrentPosition();
        double pid = controller.calculate(slidePos, target);
        double ff = Math.cos(Math.toRadians(target/ ticks_in_degrees)) * f;
        double power = pid + f;
        LinearSlide1.setPower(power);
        telemetry.addData("pos", slidePos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
