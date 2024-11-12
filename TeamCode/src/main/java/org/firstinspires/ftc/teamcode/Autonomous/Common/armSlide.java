package org.firstinspires.ftc.teamcode.Autonomous.Common;
import com.qualcomm.robotcore.util.ElapsedTime;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Autonomous.BlueAutonomous.Biddle4Specimen;

public class armSlide {
        public CRServo armServo;
        public armSlide(HardwareMap hardwareMap) {
            armServo = hardwareMap.get(CRServo.class, "servoSlide");
        }
        public class slideOut implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                armServo.setPower(0.5);
                return false;
            }
        }
        public Action slideout(){
            return new slideOut();
        }

    public class armIn implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            armServo.setPower(-0.5);
            return false;
        }
    }

    public Action armSliderIN(){
        return new armIn();
    }


    }

