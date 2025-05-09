package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(900);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 100)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -62, Math.toRadians(90)))
                .strafeTo(new Vector2d(0, -30))                 //Clipped the first one

                .strafeTo(new Vector2d(0, -40)) //Started pushing 2 In
                .splineToLinearHeading(new Pose2d(35, -34, Math.toRadians(0)), 0)
                .strafeTo(new Vector2d(35, -9))
                .strafeTo(new Vector2d(45, -9))
                .strafeTo(new Vector2d(45, -48))
                .splineToLinearHeading(new Pose2d(56, -9, Math.toRadians(0)), 0)
                .strafeTo(new Vector2d(56, -48))
                .strafeTo(new Vector2d(40, -48))
                .turn(Math.toRadians(-90)) //Finished pushing 2 in and ready to start clipping

                .strafeTo(new Vector2d(40, -62)) //Got the first one
                .strafeTo(new Vector2d(0, -50)) //Leaving after getting the speci
                .turn(Math.toRadians(180))
                .strafeTo(new Vector2d(0, -32)) //At the bar to score the speci
                .strafeTo(new Vector2d(0, -35))
                .strafeTo(new Vector2d(40, -48))
                        .turn(Math.toRadians(180))
                .strafeTo(new Vector2d(40, -62))










                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(1f)
                .addEntity(myBot)
                .start();
    }
}