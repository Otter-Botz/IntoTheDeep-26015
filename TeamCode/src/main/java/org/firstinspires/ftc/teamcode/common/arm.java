package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.interfaces.armMan;

public class arm implements armMan {
    private DcMotor slideMotor1;
    private DcMotor slideMotor2;
    @Override
    public void up() {

    }

    @Override
    public void down() {

    }

    @Override
    public void init(HardwareMap hwMap) {
        slideMotor1 = hwMap.get(DcMotor.class,  "slideMotor1");
        slideMotor2 = hwMap.get(DcMotor.class, "slideMotor2");
    }
}
