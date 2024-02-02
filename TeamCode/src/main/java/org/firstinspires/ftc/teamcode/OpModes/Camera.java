package org.firstinspires.ftc.teamcode.OpModes;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OpenCv.OpenCvPipAlbastru;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.Testing.Distractie;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Movement;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class Camera extends LinearOpMode {

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


    private static final int CAMERA_WIDTH = 1280;
    private static final int CAMERA_HEIGHT = 720;

    public enum StackPixel {
        pixelStackFront,
        pixelStackMid,
        pixelStackFar
    }

    public StackPixel stackPixel;
    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static volatile OpenCvPipAlbastru.detectie nou;

    Action pixelToBoardNT, boardToMij, exactBoard, pixelStack, pixelToPreg, mijStackPreg, goToMij;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        stackPixel = StackPixel.pixelStackFront;

        DistanceSensor DistSpateSt = hardwareMap.get(DistanceSensor.class, "DistSpateSt");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) DistSpateSt;
        DistanceSensor DistSpateDr = hardwareMap.get(DistanceSensor.class, "DistSpateDr");
        Rev2mDistanceSensor sensorTimeOfFlight1 = (Rev2mDistanceSensor) DistSpateDr;

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                logoFacingDirection, usbFacingDirection));
        imu.initialize(parameters);


        //albastru dep

        Pose2d almostBoard = new Pose2d(48, 36, 0);
        Vector2d almostBoardV = new Vector2d(48, 36);
        Pose2d boardMij = new Pose2d(53, 36, 0);
        Vector2d boardMijV = new Vector2d(53, 36);
        Pose2d boardSt = new Pose2d(53, 41, 0);
        Vector2d boardStV = new Vector2d(53, 41);
        Pose2d boardDr = new Pose2d(53, 29, 0);
        Vector2d boardDrV = new Vector2d(53, 29);
        Pose2d mij = new Pose2d(0, 11, 0);
        Vector2d mijV = new Vector2d(0, 11);
        Pose2d stackFront = new Pose2d(-58, 11, 0);
        Vector2d stackFrontV = new Vector2d(-58, 11);
        Pose2d stackMid = new Pose2d(-58, 23.5, 0);
        Vector2d stackMidV = new Vector2d(-58, 23.5);
        Pose2d stackFar = new Pose2d(-58, 35.5, 0);
        Vector2d stackFarV = new Vector2d(-58, 35.5);
        Pose2d stackPreg = new Pose2d(-40, 11, 0);
        Vector2d stackPregV = new Vector2d(-40, 11);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        goToMij = drive.actionBuilder(boardMij).strafeTo(new Vector2d(40, 12)).build();


        switch (stackPixel) {
            case pixelStackFar:
                pixelStack = drive.actionBuilder(stackPreg)
                        .setReversed(true)
                        .splineToLinearHeading(stackFar, 3)
                        .build();
                pixelToPreg = drive.actionBuilder(stackFar)
                        .strafeTo(stackPregV)
                        .build();
                break;

            case pixelStackMid:
                pixelStack = drive.actionBuilder(stackPreg)
                        .setReversed(true)
                        .splineToLinearHeading(stackMid, 3)
                        .build();
                pixelToPreg = drive.actionBuilder(stackMid)
                        .strafeTo(stackPregV)
                        .build();
                break;

            case pixelStackFront:
                pixelStack = drive.actionBuilder(stackPreg)
                        .strafeTo(new Vector2d(-58, 11))
                        .build();
                pixelToPreg = drive.actionBuilder(stackFront)
                        .strafeTo(stackPregV)
                        .build();
                break;
        }


        Lift lift = new Lift();
        Movement movement = new Movement();
        Intake intake = new Intake();
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

        int v[] = new int[4];
        v[1] = 0;
        v[2] = 0;
        v[3] = 0;

        while (opModeInInit() && !isStopRequested()) {
            nou = OpenCvPipAlbastru.getLocugasit();
            if (nou == OpenCvPipAlbastru.detectie.Dreapta) v[1]++;
            else if (nou == OpenCvPipAlbastru.detectie.Stanga) v[2]++;
            else v[3]++;
            intake.intakePos(0.2);
            telemetry.addData("Detect", nou);
            telemetry.update();
        }
        controlHubCam.stopStreaming();

        mijStackPreg = drive.actionBuilder(mij).strafeTo(stackPregV).build();

        if (v[1] > v[2] && v[1] > v[3]) {
            pixelToBoardNT = drive.actionBuilder(beginPose)
                    .strafeTo(new Vector2d(-49, 32))
                    .strafeTo(new Vector2d(-38, 36))
                    .turnTo(0)
                    .strafeTo(new Vector2d(48, 36))
                    .build();
//            drive.updatePoseEstimate();
            exactBoard = drive.actionBuilder(almostBoard)
                    .strafeTo(boardDrV)
//                    .waitSeconds(3)
                    .build();
//            drive.updatePoseEstimate();
            boardToMij = drive.actionBuilder(boardDr)
                    .strafeTo(mijV)
                    .build();
//            drive.updatePoseEstimate();
        } else if (v[3] > v[1] && v[3] > v[2]) {
            pixelToBoardNT = drive.actionBuilder(beginPose)
                    .strafeTo(new Vector2d(-38, 32))
                    .strafeTo(new Vector2d(-38, 36))
                    .turnTo(0)
                    .strafeTo(new Vector2d(48, 36))
                    .build();
//            drive.updatePoseEstimate();
            exactBoard = drive.actionBuilder(almostBoard)
                    .strafeTo(boardMijV)
//                    .waitSeconds(3)
                    .build();
//            drive.updatePoseEstimate();
            boardToMij = drive.actionBuilder(boardMij)
                    .strafeTo(mijV)
                    .build();
//            drive.updatePoseEstimate();
        } else {
            pixelToBoardNT = drive.actionBuilder(beginPose)
                    .splineTo(new Vector2d(-28, 36), -Math.PI / 4)
                    .strafeTo(new Vector2d(-38, 36))
                    .turnTo(0)
                    .strafeTo(new Vector2d(48, 36))
                    .build();
//            drive.updatePoseEstimate();
            exactBoard = drive.actionBuilder(almostBoard)
                    .strafeTo(boardStV)
//                    .waitSeconds(3)
                    .build();
//            drive.updatePoseEstimate();
            boardToMij = drive.actionBuilder(boardSt)
                    .strafeTo(mijV)
                    .build();
//            drive.updatePoseEstimate();
        }




        waitForStart();

        if (isStopRequested()) return;


        if (opModeIsActive()) {

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


            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    pixelToBoardNT,
                                    (telemetryPacket) -> {
                                        telemetry.addData("x", drive.pose.position.x);
                                        telemetry.addData("y", drive.pose.position.y);
                                        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                                        telemetry.update();
                                        return false;
                                    }),
                            new ParallelAction(
                                    exactBoard,
                                    (telemetryPacket) -> {
                                        timer.reset();
                                        lift.goTarget(2000);
                                        lift.update();

                                        while (timer.seconds() < 2) {
                                        }
                                        lift.goTarget(-50);
                                        lift.update();
                                        while (timer.seconds() < 4) {
                                        }


                                        telemetry.addData("x", drive.pose.position.x);
                                        telemetry.addData("y", drive.pose.position.y);
                                        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                                        telemetry.update();
                                        return false;
                                    }),


                            new ParallelAction(
                                    goToMij,
                                    (telemetryPacket) -> {
                                        telemetry.addData("x", drive.pose.position.x);
                                        telemetry.addData("y", drive.pose.position.y);
                                        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                                        telemetry.update();
                                        return false;
                                    })
//                            new ParallelAction(
//                                    pixelStack,
//                                    (telemetryPacket) -> {
//                                        telemetry.addData("x", drive.pose.position.x);//                                        telemetry.addData("y", drive.pose.position.y);
//                                        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
//                                        telemetry.update();
//                                        return false;
//                                    })
//                            new ParallelAction(
//                                    pixelToPreg,
//                                    (telemetryPacket) -> {
//                                        telemetry.addData("x", drive.pose.position.x);
//                                        telemetry.addData("y", drive.pose.position.y);
//                                        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
//                                        telemetry.update();
//                                        return false;
//                                    })

//                            pixelToPreg
//                            drive.actionBuilder( new Pose2d(48, 46, 0)).strafeTo(new Vector2d(53,36)),


                    ));


//            Actions.runBlocking(drive.actionBuilder(almostBoard).strafeTo(new Vector2d(40, 10)).build());



            while (DistSpateSt.getDistance(DistanceUnit.CM) > 15 && DistSpateDr.getDistance(DistanceUnit.CM) > 15) {
                double unghi = 90 - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                leftFront.setPower(-0.5 - Range.clip(unghi / 2, -0.5, 0.5)/3);
                leftRear.setPower(-0.5 - Range.clip(unghi / 2, -0.5, 0.5)/3);
                rightRear.setPower(-0.5 + Range.clip(unghi / 2, -0.5, 0.5)/3);
                rightFront.setPower(-0.5 + Range.clip(unghi / 2, -0.5, 0.5)/3);

                telemetry.addData("SenzorDr", DistSpateDr.getDistance(DistanceUnit.CM));
                telemetry.addData("SenzorSt", DistSpateSt.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
            movement.forward(0);


        }
//        Actions.runBlocking(drive.actionBuilder( new Pose2d(48, 46, 0))
//                .strafeTo(new Vector2d(53,36))
//
//        );
//        controlHubCam.stopStreaming();
        switch (nou) {

            case Dreapta:
                break;
            case Stanga:
                break;
            case Mijloc:
                break;
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
}
