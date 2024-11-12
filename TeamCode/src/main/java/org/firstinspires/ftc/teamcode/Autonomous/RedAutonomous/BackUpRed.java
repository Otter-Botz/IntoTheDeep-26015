package org.firstinspires.ftc.teamcode.Autonomous.RedAutonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.teamcode.Autonomous.Common.AutoMainSliders;
import org.firstinspires.ftc.teamcode.Autonomous.Common.armSlide;
import org.firstinspires.ftc.teamcode.Autonomous.Common.autoArmSlider;
import org.firstinspires.ftc.teamcode.Autonomous.Common.autoClaw;
import org.firstinspires.ftc.teamcode.Autonomous.Common.autoTouch;
import org.firstinspires.ftc.teamcode.Autonomous.Common.autoWrist;

public class BackUpRed {

    org.firstinspires.ftc.teamcode.Autonomous.Common.autoClaw autoClaw = new autoClaw(hardwareMap);
    autoArmSlider armMotor = new autoArmSlider(hardwareMap);
    autoWrist wristServo = new autoWrist(hardwareMap);
    armSlide slideServo = new armSlide(hardwareMap);
    autoClaw clawServo = new autoClaw(hardwareMap);
    autoTouch touchreset = new autoTouch(hardwareMap);
    org.firstinspires.ftc.teamcode.Autonomous.Common.AutoMainSliders AutoMainSliders = new AutoMainSliders(hardwareMap);

}
