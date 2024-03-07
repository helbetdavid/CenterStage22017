package org.firstinspires.ftc.teamcode.OpModes;


import static java.sql.Types.NULL;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OpenCv.OpenCvPipAlbastru;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.Testing.Distractie;
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

@Autonomous
@Config
public class AlbastruDep extends LinearOpMode {

    public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
    Pose2d beginPose = new Pose2d(-38, 61, -Math.PI / 2);
    IMU imu;

//    public class Drive implements Action{
//        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
//
//        public boolean run(@NonNull TelemetryPacket packet) {
//            drive.updatePoseEstimate();
//            return false;
//        }
//
//
//    }
//    public Action panaMea(){
//        return new Drive();
//    }

    public static final double distBoardWallX = 9.8425; // inch - arpox 25 centimetri
    public static final double distBoardWallY = 34; // inch - arpox 25 centimetri
    public static  double distCamGrob = 5.3992; // inch - arpox 16 centimetri

    public static double correctdX = NULL;
    public static double correctdY = NULL;
    public static double correctdDist = NULL;
    public static double correctdYaw = NULL;
    public static double prelungireY = NULL;
    public static double prelungireX = NULL;
    boolean targetFound = false;
    private static final boolean USE_WEBCAM = true;
    private AprilTagDetection desiredTag = null;
    public static int DESIRED_TAG_ID = 2;
    double lowSvPos = 0.56;
    double vit = 1;
    // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    private static final int CAMERA_WIDTH = 1280;
    private static final int CAMERA_HEIGHT = 720;

    public static double servo_pixel_sus= 0.4;
    public static double servo_pixel_jos = 0.915;
    public static double servo_intake_pos= 0.44;
    public static double servo_usa_inchis= 0.5;
    public static double servo_usa_deshis= 0.3;
    public static int numaratoare= 0;

    public static boolean dat_dru=false;
    //maxim 0.5= pozitia sus 1=pozitia jos

    public enum StackPixel {
        pixelStackFront,
        pixelStackMid,
        pixelStackFar
    }

    public StackPixel stackPixel;
    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static volatile OpenCvPipAlbastru.detectie nou;

    Action pixelToBoardNT, boardToMij, exactBoard, pixelStack, pixelToPreg, mijStackPreg, goToMij, stackToMijBetter, parking, boardToMijCorrected;

    ElapsedTime timer = new ElapsedTime();
    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        stackPixel = StackPixel.pixelStackFront;

        Lift lift = new Lift();
        Movement movement = new Movement();
        Intake intake = new Intake();

        lift.init(hardwareMap);
        movement.init(hardwareMap);
        intake.init(hardwareMap);

        DistanceSensor DistSpateSt = hardwareMap.get(DistanceSensor.class, "DistSpateSt");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) DistSpateSt;
        DistanceSensor DistSpateDr = hardwareMap.get(DistanceSensor.class, "DistSpateDr");
        Rev2mDistanceSensor sensorTimeOfFlight1 = (Rev2mDistanceSensor) DistSpateDr;


        Servo rightIntakeSv = hardwareMap.get(Servo.class, "rightIntakeSv");
        Servo leftIntakeSv = hardwareMap.get(Servo.class, "leftIntakeSv");
        Servo pixelInit = hardwareMap.get(Servo.class, "PixelStartSv");
        Servo ServoUsa = hardwareMap.get(Servo.class, "Usa");



        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                logoFacingDirection, usbFacingDirection));
        imu.initialize(parameters);


        //albastru dep

        Pose2d almostBoard = new Pose2d(48, 36, 0);
        Vector2d almostBoardV = new Vector2d(48, 36);
        Pose2d boardMij = new Pose2d(51.5, 36, 0);
        Vector2d boardMijV = new Vector2d(51.5, 36);
        Pose2d boardSt = new Pose2d(51.5, 41, 0);
        Vector2d boardStV = new Vector2d(51.5, 41);
        Pose2d boardDr = new Pose2d(51.5, 29, 0);
        Vector2d boardDrV = new Vector2d(51.5, 29);
        Pose2d mij = new Pose2d(11, 15, Math.PI);// y era y=17
        Vector2d mijV = new Vector2d(11, 14);
        Pose2d stackFront = new Pose2d(-60, 14, 0);
        Vector2d stackFrontV = new Vector2d(-59.5, 14);
        Pose2d stackMid = new Pose2d(-58, 23.5, 0);
        Vector2d stackMidV = new Vector2d(-58, 23.5);
        Pose2d stackFar = new Pose2d(-58, 35.5, 0);
        Vector2d stackFarV = new Vector2d(-59, 35.5);
        Pose2d stackPreg = new Pose2d(-40, 14, 0);
        Vector2d stackPregV = new Vector2d(-40, 14);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);




        DcMotor leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotor leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotor rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotor rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotor banda = hardwareMap.get(DcMotorEx.class, "banda");
        DcMotor intake1 = hardwareMap.get(DcMotorEx.class, "intake");




        initOpenCV(telemetry);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        int v[] = new int[4];
        v[1] = 0;
        v[2] = 0;
        v[3] = 0;
        ElapsedTime pixel = new ElapsedTime();
        pixel.startTime();
        while (opModeInInit() && !isStopRequested()) {
            if(pixel.seconds()>1) {
                nou = OpenCvPipAlbastru.getAnalysis();
                if (nou == OpenCvPipAlbastru.detectie.Dreapta) v[1]++;
                else if (nou == OpenCvPipAlbastru.detectie.Stanga) v[2]++;
                else v[3]++;
                pixel.reset();
            }
            pixelInit.setPosition(servo_pixel_jos);
            intake.intakePos(0.1);
            ServoUsa.setPosition(servo_usa_inchis);
            telemetry.addData("Detect", nou);
            telemetry.update();
        }
        controlHubCam.stopStreaming();
        controlHubCam.closeCameraDevice();





//            drive.updatePoseEstimate();



        if (USE_WEBCAM)
            setManualExposure(6, 250);

        waitForStart();

        if (isStopRequested()) return;
        initAprilTag();
        if (opModeIsActive()) {
            ElapsedTime run = new ElapsedTime();
            run.startTime();
            ElapsedTime full = new ElapsedTime();
            full.startTime();
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

//            Actions.runBlocking(
//                    new ParallelAction(
//                    drive.actionBuilder(beginPose)
//                            .strafeToLinearHeading(new Vector2d(-57.7, 35.5),0)// PRIMUL PIXEL
//                            .waitSeconds(0.15)
//                            .strafeToLinearHeading(new Vector2d(-41,25),0)
//                            .splineToLinearHeading(new Pose2d(-25,10,0),0)
//                            .splineToLinearHeading(new Pose2d(15,10,0),0)
//                            .splineToLinearHeading(new Pose2d(48,36,0),0.9) // AL DOILEA PIXEL
//                            .waitSeconds(3)
//                            .build()
//
//                    ,
//                            (telemetryPacket )->{
//                                    telemetryAprilTag();
//                                    telemetry.update();
//                                    if(run.seconds()>10)return false;
//                                    return true;
//                                }
//                    )
//            );
            while(run.seconds()<20){
                telemetryAprilTag();
                telemetry.update();
            }
//                                    telemetry.update();
//                                    if(run.seconds()>10)

//            Actions.runBlocking(
//                    new ParallelAction(
//                    drive.actionBuilder(beginPose)
//                            .strafeToLinearHeading(new Vector2d(-57.7, 35.5),0)// PRIMUL PIXEL
//                            .waitSeconds(0.15)
//                            .strafeToLinearHeading(new Vector2d(-41,25),0)
//                            .splineToLinearHeading(new Pose2d(-25,10,0),0)
//                            .splineToLinearHeading(new Pose2d(15,10,0),0)
//                            .splineToLinearHeading(new Pose2d(48.9,36,0),0.9) // AL DOILEA PIXEL
//                            .waitSeconds(2)
//                            .setReversed(true)
//                            .splineToLinearHeading(new Pose2d(30,20,0),new Rotation2d(-0.75,-0.75))
//                            .splineToLinearHeading(new Pose2d(-41 ,16,0),new Rotation2d(0,0))//x adevarat este -57.5
//                            .waitSeconds(1)
//                            .build()
//                        ,new InstantAction(() -> {
//
//                        }),
//                            (telemetryPacket )->{
//                                lift.update();
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
//                                return false;
//                            }
//                    )
//                    );
//                drive.pose = new Pose2d(-58,10,Math.toRadians(drive.pose.heading.toDouble()));
//
//            Actions.runBlocking(
//                    new ParallelAction(
//                            drive.actionBuilder(drive.pose)
//                                    .splineToLinearHeading(new Pose2d(15,10,0),0)
//                                    .splineToLinearHeading(new Pose2d(51,36,0),0.9) // AL DOILEA PIXEL
//                                    .waitSeconds(0.4)
//                                    .setReversed(true)
//                                    .splineToLinearHeading(new Pose2d(35,20,0),new Rotation2d(-0.75,-0.75))
//                                    .splineToLinearHeading(new Pose2d(-44.5 ,14,0),new Rotation2d(0,0))//x adevarat este -57.5
//                                    .waitSeconds(1)
//                                    .build()
//                            ,new InstantAction(() -> {
//
//                    }),
//                            (telemetryPacket )->{
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
//                                return false;
//                            }
//                    )
//            );
//            drive.pose = new Pose2d(-58.5,10,Math.toRadians(drive.pose.heading.toDouble()));
//
//            Actions.runBlocking(
//                    new ParallelAction(
//                            drive.actionBuilder(drive.pose)
//                                    .splineToLinearHeading(new Pose2d(15,10,0),0)
//                                    .splineToLinearHeading(new Pose2d(52.5,36,0),0.9) // AL DOILEA PIXEL
//                                    .waitSeconds(0.4)
//                                    .build()
//                            ,new InstantAction(() -> {
//
//                    }),
//                            (telemetryPacket )->{
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
//                                return false;
//                            }
//                    )
//            );
//            while(full.seconds()<30){
//                lift.update();
//            }
//            Actions.runBlocking(drive.actionBuilder(almostBoard).strafeTo(new Vector2d(40, 10)).build());



//            while (DistSpateSt.getDistance(DistanceUnit.CM) > 15 && DistSpateDr.getDistance(DistanceUnit.CM) > 15) {
//                double unghi = 90 - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//                leftFront.setPower(-0.5 - Range.clip(unghi / 2, -0.5, 0.5)/3);
//                leftRear.setPower(-0.5 - Range.clip(unghi / 2, -0.5, 0.5)/3);
//                rightRear.setPower(-0.5 + Range.clip(unghi / 2, -0.5, 0.5)/3);
//                rightFront.setPower(-0.5 + Range.clip(unghi / 2, -0.5, 0.5)/3);
//
//                telemetry.addData("SenzorDr", DistSpateDr.getDistance(DistanceUnit.CM));
//                telemetry.addData("SenzorSt", DistSpateSt.getDistance(DistanceUnit.CM));
//                telemetry.update();
//            }
//            movement.forward(0);


        }
//        Actions.runBlocking(drive.actionBuilder( new Pose2d(48, 46, 0))
//                .strafeTo(new Vector2d(53,36))
//
//        );
//        controlHubCam.stopStreaming();

    }

    private void initOpenCV(Telemetry telemetry) {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        OpenCvPipAlbastru OpenCvPipAlbastru = new OpenCvPipAlbastru(telemetry);

        controlHubCam.setPipeline(OpenCvPipAlbastru);
        controlHubCam.openCameraDevice();

        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
    private void telemetryAprilTag() {
        telemetry.addLine("ama ajuns acci");
        telemetry.update();
        targetFound = false;
        desiredTag = null;
        drive.updatePoseEstimate();
        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            telemetry.addLine("ama ajuns acci2");
            telemetry.update();
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    telemetry.addLine("ama ajuns acci3");
                    telemetry.update();
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
        telemetry.addLine("ama ajuns acci4");
        telemetry.update();
        currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        if (targetFound) {
//            telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
//            telemetry.addData("Dist ", desiredTag.ftcPose.range);
//            telemetry.addData("field postion?  ", desiredTag.metadata.fieldPosition);
//            telemetry.addData("position? ", desiredTag.ftcPose);
            telemetry.addData("\nBearing", desiredTag.ftcPose.bearing);
//            telemetry.addData("Yaw ", desiredTag.ftcPose.yaw);

            correctdDist = desiredTag.ftcPose.range;
            correctdYaw = Math.toRadians(desiredTag.ftcPose.bearing) + drive.pose.heading.toDouble();

            prelungireX = Math.cos(Math.abs(drive.pose.heading.toDouble())) * distCamGrob;
            prelungireY = Math.sin(Math.abs(drive.pose.heading.toDouble())) * distCamGrob;

            correctdY = 70 - (Math.sin(correctdYaw) * (correctdDist) + distBoardWallY + prelungireY);

            correctdX = 70 - (distBoardWallX + (correctdDist) * Math.cos(correctdYaw)+ prelungireX);

            telemetry.addData("\nCosinus ", Math.cos(correctdYaw));
            telemetry.addData("HEading robot",  drive.pose.heading.toDouble());
            telemetry.addData("Sinus 30 ", Math.sin(correctdYaw));
            telemetry.addData("Unghi Corectat ", correctdYaw);
            telemetry.addData("Unghi Corectat in grade ", Math.toDegrees(correctdYaw));
            telemetry.addData("Unghi Corectat in grade in cos ", Math.cos(Math.toDegrees(correctdYaw)));

            telemetry.addData("\nDist Arpil ", desiredTag.ftcPose.range);

            telemetry.addData("\nPrelungireX ", prelungireX);
            telemetry.addData("PrelungireY ", prelungireY);
            telemetry.addData("CorrectedX ", correctdX);
            telemetry.addData("CorrectedY ", correctdY);
            telemetry.addLine("\n");
            telemetry.update();

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
        telemetry.update();

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


