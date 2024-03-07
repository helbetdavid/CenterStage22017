//package org.firstinspires.ftc.teamcode.OpModes;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//
//public class idk {
//    Actions.runBlocking(
//            new ParallelAction(
//            drive.actionBuilder(beginPose)
//                            .strafeToLinearHeading(new Vector2d(-57.7, 35.5),0)// PRIMUL PIXEL
//            .waitSeconds(0.15)
//                            .strafeToLinearHeading(new Vector2d(-41,25),0)
//            .splineToLinearHeading(new Pose2d(-25,10,0),0)
//            .splineToLinearHeading(new Pose2d(15,10,0),0)
//            .splineToLinearHeading(new Pose2d(48.9,36,0),0.9) // AL DOILEA PIXEL
//            .waitSeconds(2)
//                            .setReversed(true)
//                            .splineToLinearHeading(new Pose2d(30,20,0),new Rotation2d(-0.75,-0.75))
//            .splineToLinearHeading(new Pose2d(-41 ,16,0),new Rotation2d(0,0))//x adevarat este -57.5
//            .waitSeconds(1)
//                            .build()
//                        ,new InstantAction(() -> {
//
//    }),
//            (telemetryPacket )->{
//        lift.update();
////                                if(drive.pose.position.y<22 && drive.pose.position.x>-40.5)pixelInit.setPosition(servo_pixel_sus);//x>-47 pentru dreapta sau 42 LOL sau 40/41 pentru mijloc
////                                if(drive.pose.position.x<-35 && drive.pose.position.y<50){
////                                    leftIntakeSv.setPosition(servo_intake_pos);
////                                    rightIntakeSv.setPosition(servo_intake_pos);
////                                    banda.setPower(0.7);
////                                    intake1.setPower(1);
////
////                                }
////                                else {
////
////                                    intake1.setPower(0);
////                                }
////                                if(drive.pose.position.x>35){
////                                    lift.goTarget(1200);
////                                    banda.setPower(0);
////                                }
////                                    else {
////                                        lift.goTarget(0);
////                                    ServoUsa.setPosition(servo_usa_inchis);
////                                }
////                                if(drive.pose.position.x>47){
////                                    ServoUsa.setPosition(servo_usa_deshis);
////                                }
////                                if(run.seconds()>11){
////                                    run.reset();
////                                    return false;
////                                }
//
////                                telemetry.update();
////                                return true;
//
//        return false;
//    }
//                    )
//                            );
//    drive.pose = new Pose2d(-58,10,Math.toRadians(drive.pose.heading.toDouble()));
//
//            Actions.runBlocking(
//                    new ParallelAction(
//            drive.actionBuilder(drive.pose)
//                                    .splineToLinearHeading(new Pose2d(15,10,0),0)
//            .splineToLinearHeading(new Pose2d(51,36,0),0.9) // AL DOILEA PIXEL
//            .waitSeconds(0.4)
//                                    .setReversed(true)
//                                    .splineToLinearHeading(new Pose2d(35,20,0),new Rotation2d(-0.75,-0.75))
//            .splineToLinearHeading(new Pose2d(-44.5 ,14,0),new Rotation2d(0,0))//x adevarat este -57.5
//            .waitSeconds(1)
//                                    .build()
//                            ,new InstantAction(() -> {
//
//    }),
//            (telemetryPacket )->{
////                                lift.update();
////                                if(drive.pose.position.y<20)pixelInit.setPosition(servo_pixel_sus);
////                                if(drive.pose.position.x<-35){
////                                    leftIntakeSv.setPosition(servo_intake_pos+0.06);
////                                    rightIntakeSv.setPosition(servo_intake_pos+0.06);
////                                    banda.setPower(0.8);
////                                    intake1.setPower(1);
////
////                                }
////                                else {
////
////                                    intake1.setPower(0);
////                                }
////                                if(drive.pose.position.x>38){
////                                    lift.goTarget(1600);
////                                    banda.setPower(0);
////                                }
////
////                                else {
////                                    lift.goTarget(0);
////                                    ServoUsa.setPosition(servo_usa_inchis);
////                                }
////                                if(drive.pose.position.x>48){
////                                    ServoUsa.setPosition(servo_usa_deshis);
////                                }
////                                if(run.seconds()>11){
////                                    run.reset();
////                                    return false;
////                                }
////                                //return false;
////                                telemetry.update();
////                                return true;
//        return false;
//    }
//                    )
//                            );
//    drive.pose = new Pose2d(-58.5,10,Math.toRadians(drive.pose.heading.toDouble()));
//
//            Actions.runBlocking(
//                    new ParallelAction(
//            drive.actionBuilder(drive.pose)
//                                    .splineToLinearHeading(new Pose2d(15,10,0),0)
//            .splineToLinearHeading(new Pose2d(52.5,36,0),0.9) // AL DOILEA PIXEL
//            .waitSeconds(0.4)
//                                    .build()
//                            ,new InstantAction(() -> {
//
//    }),
//            (telemetryPacket )->{
////                                lift.update();
////                                if(drive.pose.position.y<20)pixelInit.setPosition(servo_pixel_sus);
////                                if(drive.pose.position.x<-35){
////                                    leftIntakeSv.setPosition(servo_intake_pos);
////                                    rightIntakeSv.setPosition(servo_intake_pos);
////                                    banda.setPower(0.8);
////                                    intake1.setPower(1);
////
////                                }
////                                else {
////
////                                    intake1.setPower(0);
////                                }
////                                if(drive.pose.position.x>35){
////                                    lift.goTarget(1600);
////                                    banda.setPower(0);
////                                }
////                                else {
////                                    lift.goTarget(0);
////                                    ServoUsa.setPosition(servo_usa_inchis);
////                                }
////                                if(drive.pose.position.x>48.5){
////                                    ServoUsa.setPosition(servo_usa_deshis);
////                                    return false;
////                                }
////                                //return false;
////                                telemetry.update();
////                                return true;
//        return false;
//    }
//                    )
//                            );
//            while(full.seconds()<30){
//        lift.update();
//    }
//}
