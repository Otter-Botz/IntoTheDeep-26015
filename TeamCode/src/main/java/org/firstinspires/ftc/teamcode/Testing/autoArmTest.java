package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.ArmSlider;
@Autonomous

public class autoArmTest extends LinearOpMode {

    ArmSlider ArmSlider = new ArmSlider();


    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        ArmSlider.init(hardwareMap);


    }
}
