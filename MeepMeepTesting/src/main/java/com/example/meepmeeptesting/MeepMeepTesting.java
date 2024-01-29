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

        myBot.runAction(myBot.getDrive().actionBuilder(new  Pose2d(-38, 61, -Math.PI / 2))
                .strafeTo(new Vector2d(-49,32))
                .strafeTo(new Vector2d(-49,36))
                .strafeToLinearHeading(new Vector2d(48,36),0)
                .strafeTo(new Vector2d(53,29))
                .strafeTo(new Vector2d(11,11))
                .strafeTo(new Vector2d(-58,11))
//                .strafeTo(new Vector2d(-58,23.5))
                .strafeTo(new Vector2d(-58,35.5))
                .waitSeconds(1)
                .splineTo(new Vector2d(-40,11),0)
//                .lineToY(11)
//                .lineToX(-58)
//                .lineToY(20)
                .build());





        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}