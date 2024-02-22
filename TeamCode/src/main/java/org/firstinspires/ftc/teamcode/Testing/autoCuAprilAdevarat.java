package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.OpenCv.OpenCvPipAlbastru;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Movement;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous

public class autoCuAprilAdevarat extends LinearOpMode {


    //OpenCv
    private OpenCvCamera controlHubCam;
    private static final boolean USE_WEBCAM = true;

    private static final int CAMERA_WIDTH = 1280;
    private static final int CAMERA_HEIGHT = 720;
    private static volatile OpenCvPipAlbastru.detectie nou;


    // AprilTag
    public static int DESIRED_TAG_ID;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;


    //VARIABILE APRIL-TAG
    public boolean targetFound = false;
    public double aprilDrive = 0;
    public double strafe = 0;
    public double turn = 0;


    //variabile de se pot tuna ca sa ajungi calumea la aprilTag
    public static double DESIRED_DISTANCE = 10;
    public static double SPEED_GAIN, STRAFE_GAIN, TURN_GAIN, MAX_AUTO_SPEED, MAX_AUTO_STRAFE, MAX_AUTO_TURN;


    //Motoare folosite la aprilTag
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;


    //IMU
    IMU imu;
    public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    //RR
    Pose2d beginPose = new Pose2d(-38, 61, -Math.PI / 2);
    Action mergi;

    //TIMER
    ElapsedTime timer = new ElapsedTime();



    @Override
    public void runOpMode() throws InterruptedException {

        //SubSystems
        Lift lift = new Lift();
        Movement movement = new Movement();
        Intake intake = new Intake();

        lift.init(hardwareMap);
        movement.init(hardwareMap);
        intake.init(hardwareMap);


        //Motoare AprilTag
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftRear");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");

        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);



        //IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                logoFacingDirection, usbFacingDirection));
        imu.initialize(parameters);


        //RR
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        //Traiectorii RR

        Vector2d stackFrontV = new Vector2d(-60, 15);
        Vector2d stackMidV = new Vector2d(-58, 23.5);

//        Pose2d almostBoard = new Pose2d(48, 36, 0);
//        Vector2d almostBoardV = new Vector2d(48, 36);
//        Pose2d boardMij = new Pose2d(51.5, 36, 0);
//        Vector2d boardMijV = new Vector2d(51, 36);
//        Pose2d boardSt = new Pose2d(51, 40, 0);
//        Vector2d boardStV = new Vector2d(52, 40);
//        Pose2d boardDr = new Pose2d(52, 29, 0);
//        Vector2d boardDrV = new Vector2d(51.5, 29);
//        Pose2d mij = new Pose2d(11, 12, Math.PI);// y era y=17
//        Vector2d mijV = new Vector2d(11, 14);
//        Pose2d stackFront = new Pose2d(-58, 12, 0);

//        Pose2d stackMid = new Pose2d(-59, 23.5, 0);
//        Pose2d stackFar = new Pose2d(-58, 35.5, 0);
//        Vector2d stackFarV = new Vector2d(-58, 35.5);
//        Pose2d stackPreg = new Pose2d(-40, 12, 0);
//        Vector2d stackPregV = new Vector2d(-40, 16);



        //OpenCV
        initOpenCV(telemetry);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        int[] v = new int[4];
        v[1] = 0;
        v[2] = 0;
        v[3] = 0;


        //INIȚIALIZARE OPENCV

        while (opModeInInit() && !isStopRequested()) {

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



        //Oprire detectare openCV
        controlHubCam.stopStreaming();
        controlHubCam.closeCameraDevice();


        //INIȚIALIZARE APRILTAG
        initAprilTag();


        //Dashboard pentru AprilTag
        FtcDashboard dashboardApril = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboardApril.getTelemetry());


        if (v[1] > v[2] && v[1] > v[3]) {
            telemetry.addLine("am ajuns aici");
            telemetry.update();
            //DREAPTA

            DESIRED_TAG_ID = 3;

            MAX_AUTO_SPEED = 0.43;
            MAX_AUTO_STRAFE = 0.34;
            MAX_AUTO_TURN = 0.3;
            SPEED_GAIN = 0.054;
            STRAFE_GAIN = 0.025;
            TURN_GAIN = 0.024;

            mergi = drive.actionBuilder(beginPose)
                    .strafeToLinearHeading(new Vector2d(-54, 20), 0)
                    .strafeToLinearHeading(stackMidV, 0)
                    .strafeTo(new Vector2d(-54, 12))
                    .strafeToLinearHeading(new Vector2d(20, 10), Math.toRadians(30.5))
                    .build();

        } else if (v[3] > v[1] && v[3] > v[2]) {
            telemetry.addLine("am ajuns aici");
            telemetry.update();

            //MIJLOC

            DESIRED_TAG_ID = 2;

            MAX_AUTO_SPEED = 0.4;
            MAX_AUTO_STRAFE = 0.3;
            MAX_AUTO_TURN = 0.25;
            SPEED_GAIN = 0.052;
            STRAFE_GAIN = 0.03;
            TURN_GAIN = 0.0207;

            mergi = drive.actionBuilder(beginPose)
                    .strafeToLinearHeading(new Vector2d(-46, 19), 0)
                    .strafeToLinearHeading(new Vector2d(-46, 11), Math.PI / 2)
                    .strafeToLinearHeading(stackFrontV, 0)
                    .strafeToSplineHeading(new Vector2d(20, 10), Math.toRadians(0))
                    .strafeTo(new Vector2d(20, 16))
                    .turnTo(Math.toRadians(25))
                    .build();

        } else {
            telemetry.addLine("am ajuns aici");
            telemetry.update();

            //STANGA

            DESIRED_TAG_ID = 1;

            MAX_AUTO_SPEED = 0.42;
            MAX_AUTO_STRAFE = 0.34;
            MAX_AUTO_TURN = 0.3;
            SPEED_GAIN = 0.054;
            STRAFE_GAIN = 0.027;
            TURN_GAIN = 0.03;

            mergi = drive.actionBuilder(beginPose)
                    .strafeToSplineHeading(new Vector2d(-34, 26), Math.toRadians(-30))
                    .strafeToLinearHeading(stackFrontV, 0)
                    .strafeToLinearHeading(new Vector2d(20, 10), Math.toRadians(31))
                    .build();

        }


        // Tot pentru AprilTag
        // Ajuta la claritatea camerei
        if (USE_WEBCAM) setManualExposure(6, 250);



        waitForStart();

        if(opModeIsActive() && !isStopRequested()) {

            telemetry.addData("Dreapta", v[1]);
            telemetry.addData("Stanga", v[2]);
            telemetry.addData("Mijloc", v[3]);
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", Math.toDegrees(drive.pose.heading.toDouble()));

            if (nou == OpenCvPipAlbastru.detectie.Dreapta) telemetry.addLine("Dreapta");
            else if (nou == OpenCvPipAlbastru.detectie.Stanga) telemetry.addLine("Stanga");
            else telemetry.addLine("Mijloc");
            telemetry.update();




            Actions.runBlocking(mergi);


            while(!isStopRequested() && desiredTag.ftcPose.range> DESIRED_DISTANCE) {
                detectAprilTag(DESIRED_TAG_ID);
                moveRobot(aprilDrive, strafe, turn);
            }

        }

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



    public void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }

    }


    private void setManualExposure(int exposureMS, int gain) {

        if (visionPortal == null) {
            return;
        }


        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }


    public void detectAprilTag(int desiredTagId) {
        targetFound = false;

        desiredTag = null;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {

            if (detection.metadata != null) {

                if ((desiredTagId < 0) || (detection.id == desiredTagId)) {

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
    }



    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double  leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

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
    }
}