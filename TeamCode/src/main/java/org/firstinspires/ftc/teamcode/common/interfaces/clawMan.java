package org.firstinspires.ftc.teamcode.common.interfaces;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface clawMan {
    void close();
    void open();
    void init(HardwareMap hwMap);
}
