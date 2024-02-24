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
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 12.6)
                .build();
//        Pose2d almostBoard = new Pose2d(48, 36, 0);
//        Vector2d almostBoardV = new Vector2d(48, 36);
//        Pose2d boardMij = new Pose2d(53, 36, 0);
//        Vector2d boardMijV = new Vector2d(53, 36);
//        Pose2d boardSt = new Pose2d(53, 41, 0);
//        Vector2d boardStV = new Vector2d(53, 41);
//        Pose2d boardDr = new Pose2d(53, 29, 0);
//        Vector2d boardDrV = new Vector2d(53, 29);
//        Pose2d mij = new Pose2d(11, 11, Math.PI);
//        Vector2d mijV = new Vector2d(11, 11);
//        Pose2d stackFront = new Pose2d(-58, 11, 0);
//        Vector2d stackFrontV = new Vector2d(-58, 11.25);
//        Pose2d stackMid = new Pose2d(-58, 23.5, 0);
//        Vector2d stackMidV = new Vector2d(-58, 23.5);
//        Pose2d stackFar = new Pose2d(-58, 42, Math.PI/9);
//        Vector2d stackFarV = new Vector2d(-58, 35.5);
//        Pose2d stackPreg = new Pose2d(-40, 11, 0);
//        Vector2d stackPregV = new Vector2d(-40, 11.25);





        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
                .splineTo(new Vector2d(30, -30), -Math.PI / 2)
                .splineTo(new Vector2d(0, -60), -Math.PI)
                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}