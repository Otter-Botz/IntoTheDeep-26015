package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;



public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(270), Math.toRadians(270), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(38, 62, 270))

                //Sample One
                .strafeToLinearHeading(new Vector2d(-48, 38), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-44, 5), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-60, 5), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-60, 55), Math.toRadians(270))
                .waitSeconds(0)
                //Second Sample
                .strafeToLinearHeading(new Vector2d(-60, 5), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-72, 5), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-72, 55), Math.toRadians(270))
                .waitSeconds(0)
                .build());
                //i




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

    }

    }

