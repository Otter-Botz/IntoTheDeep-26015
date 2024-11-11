package org.firstinspires.ftc.teamcode.Autonomous.Common;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class autoWrist {
    //auto wrist stuff

        public Servo wristServo;

    double target = 504;

    public autoWrist(HardwareMap hardwareMap) {
            wristServo = hardwareMap.get(Servo.class, "wristServo");
        }
        //wrist up
        public class wristup implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wristServo.setPosition(0.8);
                return false;
            }

        }
        public Action wristUp(){
            return new wristup();
        }

        public class wristdown implements Action{

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                 target = 0.4;
                return false;
            }
        }
        public Action wristDown(){
            return new wristdown();
        }
    }


