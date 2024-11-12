package org.firstinspires.ftc.teamcode.Autonomous.BlueAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.Common.armSlide;


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
        armSlide.slideOut slideOut ;
       armWaitTime();

    }
}