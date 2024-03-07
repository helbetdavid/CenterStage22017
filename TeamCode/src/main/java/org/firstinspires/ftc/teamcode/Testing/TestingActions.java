package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.IntakeAction;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Disabled
@TeleOp
public class TestingActions extends LinearOpMode {
    Pose2d beginPose = new Pose2d(-38, 61, -Math.PI / 2);

    Action TrajTest;

//    AprilTag aprilTag;
    @Override
    public void runOpMode() throws InterruptedException {
//         AprilTag aprilTag = new AprilTag(hardwareMap);
        IntakeAction intake = new IntakeAction(hardwareMap);


        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        TrajTest = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(-38, 50))
                .build();




        waitForStart();

        if (opModeIsActive() && !isStopRequested()){

            Actions.runBlocking(
                    new SequentialAction(
                            intake.setTargetMid(),
                            new ParallelAction(
                            TrajTest,
                            intake.Start(),
                            intake.gotToTarget()
//                            intake.Wait(),
//                            intake.Stop(),
//                            intake.setPosition(),
//                            intake.Wait(),
//                            intake.setTargetMid(),
//                            intake.gotToTarget(),
//                            intake.Wait(),
//                            intake.setTargetMigh(),
//                            intake.gotToTarget(),
//                            intake.Wait(),
//                            intake.setTargetLow(),
//                            intake.gotToTarget(),
//                            intake.Wait()

                    ),
                            intake.Stop(),
                            intake.setTargetMid(),
                            intake.gotToTarget()


                    )
                    );
        }

    }
//    public void initAprilTag() {
//        // Create the AprilTag processor by using a builder.
//        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder().build();
//
//        // Adjust Image Decimation to trade-off detection-range for detection-rate.
//        // eg: Some typical detection data using a Logitech C920 WebCam
//        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
//        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
//        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
//        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
//        // Note: Decimation can be changed on-the-fly to adapt during a match.
//        aprilTag.setDecimation(2);
//
//        // Create the vision portal by using a builder.
//        VisionPortal visionPortal = new VisionPortal.Builder()
//                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                    .addProcessor(aprilTag)
//                    .build();
//    }

}

