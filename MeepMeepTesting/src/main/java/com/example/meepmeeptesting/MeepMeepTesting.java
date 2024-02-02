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
        Pose2d almostBoard = new Pose2d(48, 36, 0);
        Vector2d almostBoardV = new Vector2d(48, 36);
        Pose2d boardMij = new Pose2d(53, 36, 0);
        Vector2d boardMijV = new Vector2d(53, 36);
        Pose2d boardSt = new Pose2d(53, 41, 0);
        Vector2d boardStV = new Vector2d(53, 41);
        Pose2d boardDr = new Pose2d(53, 29, 0);
        Vector2d boardDrV = new Vector2d(53, 29);
        Pose2d mij = new Pose2d(11, 11, Math.PI);
        Vector2d mijV = new Vector2d(11, 11);
        Pose2d stackFront = new Pose2d(-58, 11, 0);
        Vector2d stackFrontV = new Vector2d(-58, 11.25);
        Pose2d stackMid = new Pose2d(-58, 23.5, 0);
        Vector2d stackMidV = new Vector2d(-58, 23.5);
        Pose2d stackFar = new Pose2d(-58, 35.5, 0);
        Vector2d stackFarV = new Vector2d(-58, 35.5);
        Pose2d stackPreg = new Pose2d(-40, 11, 0);
        Vector2d stackPregV = new Vector2d(-40, 11.25);


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-38, 61, -Math.PI / 2))
//                        .setReversed(true)
//                .splineToLinearHeading(new Pose2d(11, 11, Math.PI),-3)
//                        .waitSeconds(3)e

                //TODO autodreata
//                .strafeTo(new Vector2d(-41,41))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-35,45,-Math.PI/2),-1)
//                .strafeTo(new Vector2d(-35,11))
//                .turnTo(0)
//                .splineToLinearHeading(new Pose2d(48,36,0),1)
//
//                .strafeTo(boardDrV)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(11, 11.25, Math.PI),-3)
//                .strafeTo(stackPregV)
//                .turn(Math.PI)
//                .strafeTo(stackFrontV)
//                .strafeTo(stackPregV)
//                .splineToLinearHeading(new Pose2d(50,29,0),0.75)


                //TODO mijloc
//                .strafeToLinearHeading(new Vector2d(-48, 19),0)
//                .strafeToLinearHeading(new Vector2d(-59, 13),Math.PI/2)
//                                .turnTo(0)
//                .splineToLinearHeading(new Pose2d(48,36,0),0.75)
//                .setReversed(true)
//                .splineToLinearHeading(mij,-3)


                //

                //TODO STANGA
                .splineTo(new Vector2d(-28, 38), -Math.PI / 4)
                .strafeToLinearHeading(new Vector2d(-40, 11),0)
                                .strafeTo(new Vector2d(11,11))
                                .strafeTo(new Vector2d(48,36))
//                .turnTo(0)
//                        .setReversed(true)
//                .splineToLinearHeading(new Pose2d(48,36,0),1)
//                        .splineTo(new Vector2d(48,36),1)
                .build());






        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}