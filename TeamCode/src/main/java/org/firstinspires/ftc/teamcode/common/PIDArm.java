package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface PIDArm {
    void set(double position);
    double getPosition();
    void init(HardwareMap hwMap);
}
