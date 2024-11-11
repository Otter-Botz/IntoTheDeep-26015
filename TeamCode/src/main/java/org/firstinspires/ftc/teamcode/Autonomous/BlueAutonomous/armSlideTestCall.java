package org.firstinspires.ftc.teamcode.Autonomous.BlueAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class armSlideTestCall extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timetime = new ElapsedTime();

        waitForStart();
        while (opModeIsActive()) {

            double timeGoOut = timetime.seconds();

            Biddle4Specimen.armSlider.armOut armOut;


            if (timetime.seconds() >= 3) {
                stop();
            }

        }


    }

}
