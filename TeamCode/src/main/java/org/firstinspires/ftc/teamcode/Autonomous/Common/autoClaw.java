package org.firstinspires.ftc.teamcode.Autonomous.Common;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//auto claw stuff
public class autoClaw {
    double CLAW_OPEN = 0;
    double CLAW_CLOSE = 0.4;
    public Servo clawServo;

    //init stuff
    public autoClaw(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "clawServo");
    }

    //claw open
    public class open implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            clawServo.setPosition(CLAW_OPEN);

            return false;

        }
    }



    public Action clawOpen() {
        return new open();
    }

    //claw close
    public class close implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            clawServo.setPosition(CLAW_CLOSE);
            return false;
        }
    }

    public Action clawClose() {
        return new close();
    }
}



