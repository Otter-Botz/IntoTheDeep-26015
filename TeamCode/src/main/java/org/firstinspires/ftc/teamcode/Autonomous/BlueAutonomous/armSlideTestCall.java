package org.firstinspires.ftc.teamcode.Autonomous.BlueAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;



public class armSlideTestCall extends LinearOpMode  {
    ElapsedTime time = new ElapsedTime();
    double armtime = time.seconds();

    public void armWaitTime() {
        if (time.seconds() >= 3) {
            stop();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

       armWaitTime();

    }
}