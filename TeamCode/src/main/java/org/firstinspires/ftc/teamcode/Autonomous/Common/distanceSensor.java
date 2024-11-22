package org.firstinspires.ftc.teamcode.Autonomous.Common;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class distanceSensor {
    public DistanceSensor distanceSensor;
    public boolean aligned = false;


    public distanceSensor(HardwareMap hardwareMap){
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sigmaSensor");
        PID_Arm arm = new PID_Arm(hardwareMap);
    }

    public class align implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double distance = distanceSensor.getDistance(DistanceUnit.INCH);

            if (distance == 10) {
                aligned = true;
            }
            if (distance < 10){
            }
            if (distance > 10){

            }

            return false;
        }
        public Action align(){
            return new align();
        }
    }

















}
