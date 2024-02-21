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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCv.OpenCvPipAlbastru;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.AprilTag;
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

@Autonomous

public class autoCuAprilAdevarat extends LinearOpMode {

    public boolean targetFound = false;
    public double aprilDrive = 0;
    public double strafe = 0;
    public double turn = 0;
    public static double DESIRED_DISTANCE = 10; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    public static double SPEED_GAIN = 0.052;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public static double STRAFE_GAIN = 0.03;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public static double TURN_GAIN = 0.0207;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    public static double MAX_AUTO_SPEED = 0.4;   //  Clip the approach speed to this max value (adjust for your robot)
    public static double MAX_AUTO_STRAFE = 0.3;   //  Clip the approach speed to this max value (adjust for your robot)
    public static double MAX_AUTO_TURN = 0.25;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor leftFrontDrive = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive = null;  //  Used to control the right back drive wheel

    private OpenCvCamera controlHubCam;
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    public static int DESIRED_TAG_ID;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag


    public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
    Pose2d beginPose = new Pose2d(-38, 61, -Math.PI / 2);
    IMU imu;
    private static final int CAMERA_WIDTH = 1280;
    private static final int CAMERA_HEIGHT = 720;

    public enum StackPixel {
        pixelStackFront,
        pixelStackMid,
        pixelStackFar
    }

    public StackPixel stackPixel;

    private static volatile OpenCvPipAlbastru.detectie nou;

    Action pixelToBoardNT, boardToMij, exactBoard, pixelStack, pixelToPreg, mijStackPreg, goToMij, stackToMijBetter, parking, boardToMijCorrected, mergi;      //balane sa iti dau la muie cu actionurile tale si cu denumirile lor cu tot


    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        stackPixel = StackPixel.pixelStackFront;

//        AprilTag aprilTag = new AprilTag(hardwareMap);

        //SubSystems pt lift si alea alea

        Lift lift = new Lift();
        Movement movement = new Movement();
        Intake intake = new Intake();

        lift.init(hardwareMap);
        movement.init(hardwareMap);
        intake.init(hardwareMap);

        //April Tag

        // Initialize the Apriltag Detection process

        FtcDashboard dashboardApril = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboardApril.getTelemetry());

        //Motoare AprilTag
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftRear");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");


        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                logoFacingDirection, usbFacingDirection));
        imu.initialize(parameters);

        Pose2d almostBoard = new Pose2d(48, 36, 0);
        Vector2d almostBoardV = new Vector2d(48, 36);
        Pose2d boardMij = new Pose2d(51.5, 36, 0);
        Vector2d boardMijV = new Vector2d(51, 36);
        Pose2d boardSt = new Pose2d(51, 40, 0);
        Vector2d boardStV = new Vector2d(52, 40);
        Pose2d boardDr = new Pose2d(52, 29, 0);
        Vector2d boardDrV = new Vector2d(51.5, 29);
        Pose2d mij = new Pose2d(11, 12, Math.PI);// y era y=17
        Vector2d mijV = new Vector2d(11, 14);
        Pose2d stackFront = new Pose2d(-58, 12, 0);
        Vector2d stackFrontV = new Vector2d(-60, 15);
        Pose2d stackMid = new Pose2d(-59, 23.5, 0);
        Vector2d stackMidV = new Vector2d(-58, 23.5);
        Pose2d stackFar = new Pose2d(-58, 35.5, 0);
        Vector2d stackFarV = new Vector2d(-58, 35.5);
        Pose2d stackPreg = new Pose2d(-40, 12, 0);
        Vector2d stackPregV = new Vector2d(-40, 16);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

//        switch (stackPixel) {
//            case pixelStackFar:
//                pixelStack = drive.actionBuilder(drive.pose)
//                        .setReversed(true)
//                        .splineToLinearHeading(stackFar, 3)
//                        .build();
//                pixelToPreg = drive.actionBuilder(drive.pose)
//                        .strafeTo(stackPregV)
//                        .build();
//                break;
//
//            case pixelStackMid:
//                pixelStack = drive.actionBuilder(drive.pose)
//                        .setReversed(true)
//                        .splineToLinearHeading(stackMid, 3)
//                        .build();
//                pixelToPreg = drive.actionBuilder(drive.pose)
//                        .strafeTo(stackPregV)
//                        .build();
//                break;
//
//            case pixelStackFront:
//                pixelStack = drive.actionBuilder(drive.pose)
////                        .turn(Math.PI)
//                        .strafeTo(stackFrontV)
//                        .build();
//                pixelToPreg = drive.actionBuilder(drive.pose)
//                        .strafeTo(stackPregV)
//                        .build();
//                break;
//        }

        DcMotor leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotor leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotor rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotor rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        lift.init(hardwareMap);
        movement.init(hardwareMap);
        intake.init(hardwareMap);

        initOpenCV(telemetry);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        int[] v = new int[4];
        v[1] = 0;
        v[2] = 0;
        v[3] = 0;

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
ElapsedTime timer = new ElapsedTime();
        waitForStart();

        if(opModeIsActive() && !isStopRequested()) {
            // am incercat o susta
            // nu stiu daca merge
            telemetry.addData("Dreapta", v[1]);
            telemetry.addData("Stanga", v[2]);
            telemetry.addData("Mijloc", v[3]);
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
//            telemetry.addData("sdfhdfh", drive.updatePoseEstimate());
            telemetry.addData("heading", Math.toDegrees(drive.pose.heading.toDouble()));
            if (nou == OpenCvPipAlbastru.detectie.Dreapta) telemetry.addLine("Dreapta");
            else if (nou == OpenCvPipAlbastru.detectie.Stanga) telemetry.addLine("Stanga");
            else telemetry.addLine("Mijloc");
            telemetry.update();

            controlHubCam.stopStreaming();
            controlHubCam.closeCameraDevice();

            initAprilTag();

//            controlHubCam.openCameraDevice();


            Actions.runBlocking(mergi);


        }

        while(!isStopRequested()) {
//            detectAprilTag(DESIRED_TAG_ID);
                targetFound = false;

                desiredTag = null;

                // Step through the list of detected tags and look for a matching tag
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    // Look to see if we have size info on this tag.
                    if (detection.metadata != null) {
                        //  Check to see if we want to track towards this tag.
                        if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                            // Yes, we want to use this tag.
                            targetFound = true;
                            desiredTag = detection;
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

                // Tell the driver what we see, and what to do.
                if (targetFound) {
                    telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
                    telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                    telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                    telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                    telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
                } else {
                    telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
                }

                // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
                if (gamepad1.left_bumper && targetFound) {

                    // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                    double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                    double headingError = desiredTag.ftcPose.bearing;
                    double yawError = desiredTag.ftcPose.yaw;

                    // Use the speed and turn "gains" to calculate how we want the robot to move.
                    aprilDrive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                    telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                } else {

                    // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                    aprilDrive = -gamepad1.left_stick_y / 2.0;  // Reduce drive rate to 50%.
                    strafe = -gamepad1.left_stick_x / 2.0;  // Reduce strafe rate to 50%.
                    turn = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
                    telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                }
                telemetry.update();

                // Apply desired axes motions to the drivetrain.
                moveRobot(aprilDrive, strafe, turn);
                sleep(10);
            }

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

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /**
     * Initialize the AprilTag processor.
     */
    public void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
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


    private void initOpenCV(Telemetry telemetry) {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        OpenCvPipAlbastru openCvPipAlbastru = new OpenCvPipAlbastru(telemetry);

        controlHubCam.setPipeline(openCvPipAlbastru);
        controlHubCam.openCameraDevice();

        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }

    public void detectAprilTag(int desiredTagId) {
        targetFound = false;
        desiredTag = null;


        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((desiredTagId < 0) || (detection.id == desiredTagId)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
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

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
            aprilDrive = 0;
            strafe = 0;
            turn = 0;

        }

        // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
        if (targetFound) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double headingError = desiredTag.ftcPose.bearing;
            double yawError = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            aprilDrive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            telemetry.update();
            moveRobot(aprilDrive,strafe,turn);


            telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", aprilDrive, strafe, turn);
        }

        telemetry.update();
        moveRobot(aprilDrive,strafe,turn);
        sleep(10);
    }
}



