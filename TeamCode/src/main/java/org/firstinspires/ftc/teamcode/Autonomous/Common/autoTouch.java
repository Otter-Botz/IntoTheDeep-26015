package org.firstinspires.ftc.teamcode.Autonomous.Common;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class autoTouch {
        public TouchSensor touchSensor;
        public DcMotor armMotor;

    public autoTouch(HardwareMap hardwareMap) {

    }

    public void touchReset(HardwareMap hardwareMap) {
            touchSensor = hardwareMap.get(TouchSensor.class, "sensorTouch");
            armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        }

        public class touch implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (touchSensor.isPressed()) {
                    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                return false;
            }
            public Action touchreset(){
                return new touch();
            }
        }
    }


