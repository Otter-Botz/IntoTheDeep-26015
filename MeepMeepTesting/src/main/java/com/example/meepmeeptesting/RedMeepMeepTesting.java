package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedMeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot1.runAction(myBot1.getDrive().actionBuilder(new Pose2d(15, -70, 270))
                .strafeToLinearHeading(new Vector2d(0, 38), Math.toRadians(270))
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(45, 38), Math.toRadians(270))
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(45,-38),Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(47.5, -38), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(47.5, -15), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(50, -15), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(50, -38), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(52, -15), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(54, -18), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(54, -38), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(45, -38), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(0, -38), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(45, -38), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(0, -38), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(45, -38), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(0, -38), Math.toRadians(270))
                        .waitSeconds(2)
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot1)
                .start();

    }
}
