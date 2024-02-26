package org.firstinspires.ftc.teamcode.Testing;

import static java.sql.Types.NULL;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OpenCv.OpenCvPipAlbastru;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.RR.tuning.TuningOpModes;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Movement;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp
@Config
public class TestReloc extends LinearOpMode {

    public static final double distBoardWallX = 9.8425; // inch - arpox 25 centimetri
    public static final double distBoardWallY = 34; // inch - arpox 25 centimetri
    public static final double distCamGrob = 6.2992; // inch - arpox 16 centimetri

    public static double correctdX;
    public static double correctdY;
    public static double correctdDist;
    public static double correctdYaw;
    public static double prelungireY;
    public static double prelungireX;

    public static int DESIRED_TAG_ID = 2;
    boolean targetFound = false;
    private static final boolean USE_WEBCAM = true;
    private AprilTagDetection desiredTag = null;

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    MecanumDrive drive;

    Pose2d reloc;

    private OpenCvCamera controlHubCam;
    private static final int CAMERA_WIDTH = 1280;
    private static final int CAMERA_HEIGHT = 720;
    private static volatile OpenCvPipAlbastru.detectie nou;

    Pose2d beginPose = new Pose2d(-38, 61, -Math.PI / 2);
    Action mergi;


    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, beginPose);

        Lift lift = new Lift();
        Movement movement = new Movement();
        Intake intake = new Intake();

        lift.init(hardwareMap);
        movement.init(hardwareMap);
        intake.init(hardwareMap);


        //Traiectorii RR

        Vector2d stackFrontV = new Vector2d(-60, 15);
        Vector2d stackMidV = new Vector2d(-58, 23.5);

        initOpenCV(telemetry);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        int[] v = new int[4];
        v[1] = 0;
        v[2] = 0;
        v[3] = 0;

        ElapsedTime inita = new ElapsedTime();
        while (opModeInInit() && !isStopRequested() && inita.seconds() < 5) {

            nou = OpenCvPipAlbastru.getLocugasit();
            if (nou == OpenCvPipAlbastru.detectie.Dreapta) v[1]++;
            else if (nou == OpenCvPipAlbastru.detectie.Stanga) v[2]++;
            else v[3]++;
            intake.intakePos(0.1);

            telemetry.addData("Detect", nou);
            telemetry.addData("Dreapta", v[1]);
            telemetry.addData("Stanga", v[2]);
            telemetry.addData("Mijloc", v[3]);
            telemetry.update();
        }

        //Oprire openCV
        controlHubCam.stopStreaming();
        controlHubCam.closeCameraDevice();

        initAprilTag();

        //INIȚIALIZARE APRILTAG

        if (v[1] > v[2] && v[1] > v[3]) {
            telemetry.addLine("sunt pe dreapta");
            telemetry.update();
            //DREAPTA

            mergi = drive.actionBuilder(beginPose)
                    .strafeToLinearHeading(new Vector2d(-54, 20), 0)
                    .strafeToLinearHeading(stackMidV, 0)
                    .strafeTo(new Vector2d(-54, 12))
                    .strafeToLinearHeading(new Vector2d(20, 10), Math.toRadians(30.5))
                    .build();

        } else if (v[3] > v[1] && v[3] > v[2]) {
            telemetry.addLine("sunt pe mijloc");
            telemetry.update();

            //MIJLOC

            mergi = drive.actionBuilder(beginPose)
                    .strafeToLinearHeading(new Vector2d(-46, 19), 0)
                    .strafeToLinearHeading(new Vector2d(-46, 11), Math.PI / 2)
                    .strafeToLinearHeading(stackFrontV, 0)
                    .strafeToSplineHeading(new Vector2d(22, 10), Math.toRadians(0))
                    .strafeTo(new Vector2d(22, 30))
                    .waitSeconds(0.5)
                    .build();

        } else {
            telemetry.addLine("sunt pe stanga");
            telemetry.update();

            //STANGA


            mergi = drive.actionBuilder(beginPose)
                    .strafeToSplineHeading(new Vector2d(-34, 26), Math.toRadians(-30))
                    .strafeToLinearHeading(stackFrontV, 0)
                    .strafeToLinearHeading(new Vector2d(20, 10), Math.toRadians(31))
                    .build();

        }

        if (USE_WEBCAM)
            setManualExposure(6, 250);


        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            initAprilTag();
            telemetry.addData("Dreapta", v[1]);
            telemetry.addData("Stanga", v[2]);
            telemetry.addData("Mijloc", v[3]);
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", Math.toDegrees(drive.pose.heading.toDouble()));
            ElapsedTime timer = new ElapsedTime();

            Actions.runBlocking(mergi);
            while(timer.seconds() <= 8){
                idle();
            }
            drive.updatePoseEstimate();

            targetFound = false;
            desiredTag = null;

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            while (timer.seconds() <= 11)
                // Step through the list of detected tags and look for a matching tag
                for (AprilTagDetection detection : currentDetections) {
                    // Look to see if we have size info on this tag.
                    if (detection.metadata != null) {
                        //  Check to see if we want to track towards this tag.
                        if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                            // Yes, we want to use this tag.
                            targetFound = true;
                            desiredTag = detection;
                            telemetryAprilTag();
                            break;  // don't look any further.
                        } else {
                            // This tag is in the library, but we do not want to track it right now.
                            telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                        }
                    } else {
                        // This tag is NOT in the library, so we don't have enough information to track to it.
                        telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                    }
                }

            telemetry.addData("pose ", drive.pose);
            telemetry.addData("\ny ", drive.pose.position.y);
            telemetry.addData("x ", drive.pose.position.x);
            telemetry.addData("heading (deg)", drive.pose.heading.toDouble());
//                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));

            reloc = new Pose2d(correctdX, correctdY, drive.pose.heading.toDouble());
            telemetry.addData("jhcgtfuy7yugjhv hgki", reloc);
            telemetry.update();

            Action merji2 = drive.actionBuilder(reloc)
                    .strafeToLinearHeading(new Vector2d(50, 36), 0)
                    .waitSeconds(10)
                    .build();

            Actions.runBlocking(merji2);

        }
        visionPortal.close();


    }

    private void initOpenCV(Telemetry telemetry) {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        OpenCvPipAlbastru openCvPipAlbastru = new OpenCvPipAlbastru(telemetry);

        controlHubCam.setPipeline(openCvPipAlbastru);
        controlHubCam.openCameraDevice();

        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }


    public void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        if (targetFound) {
            telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Dist ", desiredTag.ftcPose.range);
            telemetry.addData("field postion?  ", desiredTag.metadata.fieldPosition);
            telemetry.addData("position? ", desiredTag.ftcPose);
            telemetry.addData("\nBearing", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw ", desiredTag.ftcPose.yaw);

            correctdDist = desiredTag.ftcPose.range;
            correctdYaw = Math.toRadians(desiredTag.ftcPose.bearing) + Math.toRadians(drive.pose.heading.toDouble());

            prelungireX = Math.cos(Math.abs(Math.toRadians(drive.pose.heading.toDouble()))) * distCamGrob;
            prelungireY = Math.sin(Math.abs(Math.toRadians(drive.pose.heading.toDouble()))) * distCamGrob;

            correctdY = 70 - (Math.sin(correctdYaw) * (correctdDist) + distBoardWallY + prelungireY);

            correctdX = 70 - (distBoardWallX + (correctdDist) * Math.cos(correctdYaw) + prelungireX);

            telemetry.addData("\nCosinus ", Math.cos(correctdYaw));
            telemetry.addData("\nCosinus 30 ", Math.cos(30));
            telemetry.addData("Sinus ", Math.sin(correctdYaw));
            telemetry.addData("Unghi Corectat ", correctdYaw);
            telemetry.addData("Unghi Corectat v2", desiredTag.ftcPose.bearing + Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addData("Dist Arpil ", desiredTag.ftcPose.range);

            telemetry.addData("\nPrelungireX ", prelungireX);
            telemetry.addData("PrelungireY ", prelungireY);
            telemetry.addData("CorrectedX ", correctdX);
            telemetry.addData("CorrectedY ", correctdY);
            telemetry.addLine("\n");

        } else telemetry.addLine("\nNU\n");

//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//            } else {
//                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//            }
//        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

//        return new Pose2d(correctdX, correctdY, drive.pose.heading.toDouble());
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                .setLensIntrinsics(1412.38, 1412.38, 654.15, 355.152)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(aprilTag, true);

    }

    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}