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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.6)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(61, -61.5, 0))
//                .waitSeconds(5)
                .strafeTo(new Vector2d(13,-16))
                .splineTo(new Vector2d(-54,55),-Math.PI*3/2-0.15)
                .waitSeconds(4)
                .splineTo(new Vector2d(-13,-10),0)
                .splineTo(new Vector2d(52,-38),0)
//                .waitSeconds(4)
                .build());





        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}