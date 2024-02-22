package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTag {
    public static double DESIRED_DISTANCE = 0;
    public static double SPEED_GAIN = 0;
    public static double STRAFE_GAIN = 0;
    public static double TURN_GAIN = 0;

    public static double MAX_AUTO_SPEED = 0;
    public static double MAX_AUTO_STRAFE = 0;
    public static double MAX_AUTO_TURN = 0;

    public boolean targetFound = false;
    public VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    private WebcamName webCam1;
    public static int DESIRED_TAG_ID = -1;

    public double aprilDrive = 0;
    public double strafe = 0;
    public double turn = 0;

    public AprilTag(HardwareMap hwmap) {

        leftFrontDrive = hwmap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hwmap.get(DcMotor.class, "rightFront");
        leftBackDrive = hwmap.get(DcMotor.class, "leftRear");
        rightBackDrive = hwmap.get(DcMotor.class, "rightRear");

        webCam1 = hwmap.get(WebcamName.class, "Webcam 1");

        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


    }


    public class InitAprilTag implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            aprilTag = new AprilTagProcessor.Builder().build();
            aprilTag.setDecimation(2);

            visionPortal = new VisionPortal.Builder()
                    .setCamera(webCam1)
                    .addProcessor(aprilTag)
                    .build();

            return false;
        }
    }

    public class DetectAprilTag implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            targetFound = false;
            desiredTag = null;

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {

                if (detection.metadata != null) {

                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {

                        targetFound = true;
                        desiredTag = detection;
                        break;
                    } else {

                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }

                } else {

                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            if (targetFound) {

                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
            }


            if (targetFound) {

                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;


                aprilDrive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ",aprilDrive, strafe, turn);
            }
            telemetry.update();

            return false;
        }

    }

    public class GoToAprilTag implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double leftFrontPower = aprilDrive - strafe - turn;
            double rightFrontPower = aprilDrive + strafe + turn;
            double leftBackPower = aprilDrive + strafe - turn;
            double rightBackPower = aprilDrive - strafe + turn;

            // Normalize wheel powers to be less than 1.0
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }


            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            return false;
        }
    }



    public Action detectAprilTag() {
        return new DetectAprilTag();
    }

    public Action goToAprilTag() {
        return new GoToAprilTag();
    }
}