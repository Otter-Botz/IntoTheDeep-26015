package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;



public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-16, 60, 90))

                //Score Preloaded Specimen
                .strafeToLinearHeading(new Vector2d(0, 38), Math.toRadians(270))
                .waitSeconds(2)
                //Go to spike 1 and push sample to human player
                .strafeToLinearHeading(new Vector2d(-30, 38), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-45, 10), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-47, 62), Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-47, 10), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-55, 10), Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-55, 62), Math.toRadians(270))
//                .waitSeconds(0)
                //rihgtrhgtugtr8ghtriuhgiugtriugtrwiugtrugtriugtriugtuiugtgyuhyhyuhytuhgyuhytugygughyuhyuhy
//                .strafeToLinearHeading(new Vector2d(55, -10), Math.toRadians(90))
//                .waitSeconds(0)
//                .strafeToLinearHeading(new Vector2d(62, -10), Math.toRadians(90))
//                .waitSeconds(0)
//                .strafeToLinearHeading(new Vector2d(62, -72), Math.toRadians(90))
//                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(0, -72), Math.toRadians(90))
//                .waitSeconds(0)
//                .strafeToLinearHeading(new Vector2d(0, -38), Math.toRadians(90))
//                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(0, -60), Math.toRadians(90))
//                .waitSeconds(0)
//                .strafeToLinearHeading(new Vector2d(55, -60), Math.toRadians(90))
//                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(0, -60), Math.toRadians(90))
//                .waitSeconds(0)
//                .strafeToLinearHeading(new Vector2d(0, -38), Math.toRadians(90))
//                .waitSeconds(2)

                //.strafeToLinearHeading(new Vector2d(0, 38), Math.toRadians(270))
              //  .waitSeconds(2)
             //   .strafeToLinearHeading(new Vector2d(-45, 38), Math.toRadians(270))
             //   .waitSeconds(2)
                .build());
                //i




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

    }

    }

