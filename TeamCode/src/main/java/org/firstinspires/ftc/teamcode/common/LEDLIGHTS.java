package org.firstinspires.ftc.teamcode.common;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.LED;

public class LEDLIGHTS extends OpMode {

    LED REDD;
    LED GREENN;

    @Override
    public void init() {
        REDD = hardwareMap.get(LED.class, "REDD");
        GREENN = hardwareMap.get(LED.class, "GREENN");
    }

    @Override
    public void loop() {
        }

    public void OnLights(){
        REDD.on();
        GREENN.on();
    }
    public void OffLights(){
        REDD.off();
        GREENN.off();
    }
//chucky
}
