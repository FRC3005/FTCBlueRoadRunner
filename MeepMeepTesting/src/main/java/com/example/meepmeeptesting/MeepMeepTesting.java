package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-23, -62, 0))
                .strafeTo(new Vector2d(-60, -58))
                .turn(Math.toRadians(49))
                .strafeTo(new Vector2d(-34, -35))
                .turn(Math.toRadians(100))
                .strafeTo(new Vector2d(-60, -58))
                .turn(Math.toRadians(-100))
                .strafeTo(new Vector2d(-43, -31))
                .turn(Math.toRadians(100))
                .strafeTo(new Vector2d(-60, -58))
                .turn(Math.toRadians(-100))
                .strafeTo(new Vector2d(-47, -26))
                .turn(Math.toRadians(130))
                .strafeTo(new Vector2d(-60, -58))
                .turn(Math.toRadians(-130))



                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(1f)
                .addEntity(myBot)
                .start();
    }
}