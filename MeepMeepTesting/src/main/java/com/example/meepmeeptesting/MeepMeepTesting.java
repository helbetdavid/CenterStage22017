package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.6)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(53, 36, 0))

                                .strafeTo(new Vector2d(-35, -10))
                                .splineToSplineHeading(new Pose2d(-60, -60, -Math.PI * 3 / 4), -Math.PI * 3 / 4)

                                .strafeTo(new Vector2d(-40, -20))

//                                .splineToSplineHeading(new Pose2d(10, 20, 0), 0)



//                                .strafeTo(new Vector2d(-30, -10))
//                                .lineTo(new Vector2d(0,0))
                                .splineToSplineHeading(new Pose2d(0, 0, 0), 0)
                                .splineToConstantHeading(new Vector2d(53, 36), 0)
//                                .strafeTo(new Vector2d(53, 36))
//                                .splineToConstantHeading(new Vector2d(53, 36), 0)


//                                .strafeTo(new Vector2d(-49,36))

//                                .strafeTo(new Vector2d(53,33))
//                                .lineTo(new Vector2d(-41,58))
//                                .strafeTo(new Vector2d(-43,34) )
//                                .lineToLinearHeading(new Pose2d(48, 36, 0))
//                                .strafeTo(new Vector2d(53,31))
//                                .strafeTo(new Vector2d(10,11))
//                                .lineTo(new Vector2d(-50,11))
//                                .splineTo(new Vector2d(10, 10), 0)

                                                .build()
                                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}