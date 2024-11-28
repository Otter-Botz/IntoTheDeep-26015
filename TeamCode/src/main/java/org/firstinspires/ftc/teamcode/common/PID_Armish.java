package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface PID_Armish {
    void up();

    void armRespond(double value);

    void down();

    void math();

    void set(double position);

    double getPosition();

    void init(HardwareMap hwMap);
}
