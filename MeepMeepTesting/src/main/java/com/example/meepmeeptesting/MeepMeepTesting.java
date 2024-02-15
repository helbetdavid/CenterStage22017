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

        Pose2d almostBoard = new Pose2d(48, 36, 0);
        Vector2d almostBoardV = new Vector2d(48, 36);
        Pose2d boardMij = new Pose2d(53, 36, 0);
        Vector2d boardMijV = new Vector2d(53, -36);
        Pose2d boardSt = new Pose2d(53, -41, 0);
        Vector2d boardStV = new Vector2d(53, -41);
        Pose2d boardDr = new Pose2d(53, 29, 0);
        Vector2d boardDrV = new Vector2d(53, 29);
        Pose2d mij = new Pose2d(11, 11, Math.PI);
        Vector2d mijV = new Vector2d(11, 11);
        Pose2d stackFront = new Pose2d(-58, 11, 0);
        Vector2d stackFrontV = new Vector2d(-58, 11.25);
        Pose2d stackMid = new Pose2d(-58, 23.5, 0);
        Vector2d stackMidV = new Vector2d(-58, 23.5);
        Pose2d stackFar = new Pose2d(-58, 42, Math.PI/9);
        Vector2d stackFarV = new Vector2d(-58, 35.5);
        Pose2d stackPreg = new Pose2d(-40, 11, 0);
        Vector2d stackPregV = new Vector2d(-40, 11.25);




        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-34.12, 24.48, Math.PI / 2))



                .splineTo(new Vector2d(30, 30), Math.PI / 2)
                .splineTo(new Vector2d(0, 60), Math.PI)
                .build());
////                        .setReversed(true)
//                .splineToLinearHeading(new Pose2d(11, 11, Math.PI),-3)
//                        .waitSeconds(3)e


//                .strafeTo(new Vector2d(-41,-41))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-36,-48,Math.PI/2),-1)
//                .strafeTo(new Vector2d(-35,-11))
//                .turnTo(0)
//                .splineToLinearHeading(new Pose2d(48,-36,0),-1)

//                .strafeToLinearHeading(new Vector2d(-48, -16.5),0)
//                .strafeToLinearHeading(new Vector2d(-48, -10),-Math.PI/2)
//                .turnTo(0)
//                .splineToLinearHeading(new Pose2d(48,-36,0),-0.75)
//                .strafeTo(boardMijV)
//                .strafeTo(new Vector2d(42,-56))
//                .strafeTo(new Vector2d(58,-56))
                //TODO autodreata albastru
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
//                .strafeToLinearHeading(new Vector2d(-48, 11),Math.PI/2)
//                                .turnTo(0)
//                .splineToLinearHeading(new Pose2d(48,36,0),0.75)
//                                .strafeTo(boardMijV)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(11, 11.25, Math.PI),-3)
//                .strafeTo(stackPregV)
//                .turn(Math.PI)
//                .strafeTo(stackFrontV)
//                .strafeTo(stackPregV)
//                .splineToLinearHeading(new Pose2d(50,29,0),0.75)


                //

                //TODO STANGA
//                .splineTo(new Vector2d(-28, 36), -Math.PI / 4)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-36,38,0),0)
//                .strafeTo(new Vector2d(-36,11))
//                .setReversed(false)
//                .splineToLinearHeading(new Pose2d(45,36,0),1)
//                .strafeTo(boardStV)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(11, 11.25, Math.PI),-3)
//                .strafeTo(stackPregV)
//                .turn(Math.PI)
//                .strafeTo(stackFrontV)
//                .strafeTo(stackPregV)
//                .splineToLinearHeading(new Pose2d(50,29,0),0.75)



        //TODO Autonom Aproape Alabstru

                //TODO stanga
//                .splineTo(new Vector2d(-32, 36), -Math.PI / 4)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-36,38,0),0)
//                .strafeTo(new Vector2d(-36,11))
//                .setReversed(false)
//                .splineToLinearHeading(new Pose2d(45,36,0),1)


                //TODO dreapta

//                        .splineTo(new Vector2d(7,36), -Math.PI*3/4)
//
//                        .setReversed(true)
//                .splineToLinearHeading(new Pose2d(20,41,Math.PI),0)
//                        .turnTo(0)
//                        .setReversed(false)
//                        .splineTo(new Vector2d(48,36),0)
//                          .strafeTo(boardDrV)
//                .strafeTo(new Vector2d(42,59))
//                .strafeTo(new Vector2d(58,59))

//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(11, 11.25, Math.PI),-3.5)
//                .strafeTo(stackPregV)
//                .turn(Math.PI)
//                .strafeTo(stackFrontV)
//                .strafeTo(stackPregV)
//                .splineToLinearHeading(new Pose2d(50,29,0),0.75)

                //TODO mijloc

//                        .strafeToLinearHeading(new Vector2d(26,25 ),-Math.PI*3/4-0.1)
//                        .setReversed(true)
//                .splineToLinearHeading(new Pose2d(34,30,Math.PI),0)
//                        .turnTo(0)
//                        .strafeTo(new Vector2d(48,36))
////
//                .strafeTo(boardMijV)
//                                .strafeTo(new Vector2d(42,59))
//                                .strafeTo(new Vector2d(58,59))

//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(17, 57, Math.PI),-3)
//                .strafeTo(new Vector2d(-50,57))
//                        .turnTo(Math.PI/4)
//                        .setReversed(true)
//                .splineToLinearHeading(stackFar, 3)
//                        .strafeTo(new Vector2d(-61,41))
//                        .setReversed(false)
//                .splineToLinearHeading(new Pose2d(-50,57,0), 0)
//                        .strafeTo(new Vector2d(17,57))
//                .splineToLinearHeading(new Pose2d(50,32,0),0.75)



                //TODO Autonom Aproape Rosu

                //TODO stanga Rosu ap
//                .splineTo(new Vector2d(7,-37),2)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(15,-44,Math.PI*3/4),1)
//
//                                .splineToLinearHeading(new Pose2d(48,-36,0),1)
//
//                                .strafeTo(boardStV)
//                                .strafeTo(new Vector2d(42,-59))
//                                .strafeTo(new Vector2d(58,-59))


                //TODO dreapta rosu ap

//                       .strafeToLinearHeading(new Vector2d(31,-37 ),Math.PI*3/4-0.1)
//                    .setReversed(true)
//                    .splineToLinearHeading(new Pose2d(48,-36,Math.PI),-1)
//                    .turnTo(0)
//

//                .splineTo(new Vector2d(-32, -36), -Math.PI / 4)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-36,38,0),0)
//                .strafeTo(new Vector2d(-36,11))
//                .setReversed(false)
//                .splineToLinearHeading(new Pose2d(45,36,0),1)

//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(11, 11.25, Math.PI),-3.5)
//                .strafeTo(stackPregV)
//                .turn(Math.PI)
//                .strafeTo(stackFrontV)
//                .strafeTo(stackPregV)
//                .splineToLinearHeading(new Pose2d(50,29,0),0.75)

                //TODO mijloc rosu ap

//                        .strafeToLinearHeading(new Vector2d(23,-30 ),-Math.PI)
//                        .setReversed(true)
//                .splineToLinearHeading(new Pose2d(34,-30,Math.PI),0)
//                        .turnTo(0)
//                        .strafeTo(new Vector2d(48,-36))
////
//                .strafeToLinearHeading(new Vector2d(31,37 ),-Math.PI+0.25)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(48,36,-Math.PI),-1)
//                .turnTo(0)


//                .strafeToLinearHeading(new Vector2d(30,37 ),-Math.PI/2-0.4)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(45,37,-Math.PI),-1)
//                .turnTo(0)
//                .strafeTo(almostBoardV)
//                .strafeTo(new Vector2d(-41,-41))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-36,-48,Math.PI/2),-1)
//                .strafeTo(new Vector2d(-35,-11))
//                .turnTo(0)
//                .splineToLinearHeading(new Pose2d(48,-36,0),-1)

//                .strafeTo(boardMijV)
//                .strafeTo(new Vector2d(42,-59))
//                .strafeTo(new Vector2d(58,-59))

//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(17, 57, Math.PI),-3)
//                .strafeTo(new Vector2d(-50,57))
//                        .turnTo(Math.PI/4)
//                        .setReversed(true)
//                .splineToLinearHeading(stackFar, 3)
//                        .strafeTo(new Vector2d(-61,41))
//                        .setReversed(false)
//                .splineToLinearHeading(new Pose2d(-50,57,0), 0)
//                        .strafeTo(new Vector2d(17,57))
//                .splineToLinearHeading(new Pose2d(50,32,0),0.75)
//                .strafeTo(new Vector2d(-41,-41))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-36,-48,Math.PI/2),-1)
//                .strafeTo(new Vector2d(-35,-11))
//                .turnTo(0)
//                .splineToLinearHeading(new Pose2d(48,-36,0),-1)
//                .strafeToLinearHeading(new Vector2d(-48, 19),0)
//                .strafeToLinearHeading(new Vector2d(-48, 11),Math.PI/2)
//                .turnTo(0)
//                .splineToLinearHeading(new Pose2d(48,36,0),0.75)
//                .splineTo(new Vector2d(7,-37),2)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(15,-44,Math.PI*3/4),1)
//
//                .splineToLinearHeading(new Pose2d(48,-36,0),1)

//                .strafeToLinearHeading(new Vector2d(31,-37 ),Math.PI*3/4-0.1)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(48,-36,Math.PI),-1)
//                .turnTo(0)
//                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}