package org.firstinspires.ftc.teamcode.Autonoms;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OpenCv.OpenCvPipRosuAp;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.Testing.Distractie;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeAction;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Movement;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class Rosu_Dep extends LinearOpMode {

    public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
    Pose2d beginPose = new Pose2d(-38, -61, Math.PI / 2);
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
    public static double servo_pixel_sus = 0.4;
    public static double servo_pixel_jos = 0.97;
    public static double servo_intake_pos = 0.44;
    public static double servo_usa_inchis = 0.5;
    public static double servo_usa_deshis = 0.3;
    public static int numaratoare = 0;


    public enum StackPixel {
        pixelStackFront,
        pixelStackMid,
        pixelStackFar
    }

    public StackPixel stackPixel;
    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static volatile OpenCvPipRosuAp.detectie nou;

    Action pixelToBoardNT, boardToMij, exactBoard, pixelStack, pixelToPreg, mijStackPreg, goToMij, stackToMijBetter, parking, boardToMijCorrected;

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

        Pose2d almostBoard = new Pose2d(48, -36, 0);
        Vector2d almostBoardV = new Vector2d(48, - 36);
        Pose2d boardMij = new Pose2d(53, -32, 0);
        Vector2d boardMijV = new Vector2d(53, -32);
        Pose2d boardSt = new Pose2d(53, -39, 0);
        Vector2d boardStV = new Vector2d(53, -39);
        Pose2d boardDr = new Pose2d(53, -27, 0);
        Vector2d boardDrV = new Vector2d(53, -27);
        Pose2d mij = new Pose2d(11, -9, Math.PI);
        Vector2d mijV = new Vector2d(11, -11);
        Pose2d stackFront = new Pose2d(-58, -11, 0);
        Vector2d stackFrontV = new Vector2d(-58, -11.25);
        Pose2d stackMid = new Pose2d(-58, -23.5, 0);
        Vector2d stackMidV = new Vector2d(-58, -23.5);
        Pose2d stackFar = new Pose2d(-58, -42, Math.PI/9);
        Vector2d stackFarV = new Vector2d(-58, -35.5);
        Pose2d stackPreg = new Pose2d(-40, -11, 0);
        Vector2d stackPregV = new Vector2d(-40, -11.25);
        Vector2d caz = null;


        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);



        Lift lift = new Lift();
        Movement movement = new Movement();
        Intake intake = new Intake();
        IntakeAction intakenou = new IntakeAction(hardwareMap);

        DcMotor leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotor leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotor rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotor rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotor banda = hardwareMap.get(DcMotorEx.class, "banda");
        DcMotor intake1 = hardwareMap.get(DcMotorEx.class, "intake");


        lift.init(hardwareMap);
        movement.init(hardwareMap);
        intake.init(hardwareMap);


        Servo rightIntakeSv = hardwareMap.get(Servo.class, "rightIntakeSv");
        Servo leftIntakeSv = hardwareMap.get(Servo.class, "leftIntakeSv");
        Servo pixelInit = hardwareMap.get(Servo.class, "PixelStartSv");
        Servo ServoUsa = hardwareMap.get(Servo.class, "Usa");

        initOpenCV(telemetry);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        int v[] = new int[4];
        v[1] = 0;
        v[2] = 0;
        v[3] = 0;

        while (opModeInInit() && !isStopRequested()) {
            nou = OpenCvPipRosuAp.getAnalysis();
            if (nou == OpenCvPipRosuAp.detectie.Dreapta) v[1]++;
            else if(nou==null);
            else if (nou == OpenCvPipRosuAp.detectie.Stanga) v[2]++;
            else v[3]++;
            intake.intakePos(0.1);
            telemetry.addData("Detect", nou);
            telemetry.addData("Dreapta", v[1]);
            telemetry.addData("Stanga", v[2]);
            telemetry.addData("Mijloc", v[3]);
            telemetry.update();
        }
        controlHubCam.stopStreaming();


        waitForStart();

        if (isStopRequested()) return;


        if (opModeIsActive()) {
            ElapsedTime run = new ElapsedTime();
            run.startTime();
            ElapsedTime total = new ElapsedTime();
            total.startTime();
            telemetry.addData("Dreapta", v[1]);
            telemetry.addData("Stanga", v[2]);
            telemetry.addData("Mijloc", v[3]);
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
//            telemetry.addData("sdfhdfh", drive.updatePoseEstimate());
            telemetry.addData("heading", Math.toDegrees(drive.pose.heading.toDouble()));
            if (nou == OpenCvPipRosuAp.detectie.Dreapta) telemetry.addLine("Dreapta");
            else if (nou == OpenCvPipRosuAp.detectie.Stanga) telemetry.addLine("Stanga");
            else telemetry.addLine("Mijloc");
            telemetry.update();


            // DE VAZUT REGIO
            if(v[3]>v[1] && v[3]>v[1]){ //Mijloc
                caz = new Vector2d(-39,-20);
            }
            else if(v[2]>v[1] && v[2]>v[3]){ //Stanga
                //                Vector2d caz = new Vector2d()
            }
            else{ //Dreapta
                caz = new Vector2d(-42,-22);
            }

            Actions.runBlocking(
                    new ParallelAction(
                            drive.actionBuilder(beginPose)
                                    .strafeToLinearHeading(new Vector2d(-57.7, -35.5), 0)// PRIMUL PIXEL
                                    .waitSeconds(0.15)
                                    .strafeToLinearHeading(caz, 0)
                                    .stopAndAdd(intakenou.setLatSus())
                                    .splineToLinearHeading(new Pose2d(-25, -10, 0), 0)
                                    .splineToLinearHeading(new Pose2d(15, -10, 0), 0)
                                    .splineToLinearHeading(new Pose2d(49.5, -36, 0), -0.9)
                                    .stopAndAdd(intakenou.setUsaOpen())// AL DOILEA PIXEL
                                    .waitSeconds(0.1)
                                    .setReversed(true)
                                    .splineToLinearHeading(new Pose2d(30, -20, 0), new Rotation2d(-0.75, 0.75))
                                    .splineToLinearHeading(new Pose2d(-45, -16, 0), new Rotation2d(0, 0))//x adevarat este -57.5
                                    .waitSeconds(1)
                                    .build()
                            , new InstantAction(() -> {

                    }),
                            (telemetryPacket) -> {
                                lift.update();
                                if (drive.pose.position.x < -35 && drive.pose.position.y > -50) {
                                    leftIntakeSv.setPosition(servo_intake_pos);
                                    rightIntakeSv.setPosition(servo_intake_pos);
                                    banda.setPower(0.7);
                                    intake1.setPower(1);

                                } else {

                                    intake1.setPower(0);
                                }
                                if (drive.pose.position.x > 35) {
                                    lift.goTarget(1200);
                                    banda.setPower(0);
                                } else {
                                    lift.goTarget(0);
                                    ServoUsa.setPosition(servo_usa_inchis);
                                }
                                if (drive.pose.position.x > 47) {
                                    ServoUsa.setPosition(servo_usa_deshis);
                                }
                                if (run.seconds() > 11) {
                                    run.reset();
                                    return false;
                                }

                                telemetry.update();
                                return true;
                            }
                    )
            );
            drive.pose = new Pose2d(-60, -10, Math.toRadians(drive.pose.heading.toDouble()));

            Actions.runBlocking(
                    new ParallelAction(
                            drive.actionBuilder(drive.pose)
                                    .splineToLinearHeading(new Pose2d(15, -10, 0), 0)
                                    .splineToLinearHeading(new Pose2d(51, -36, 0), -0.9) // AL DOILEA PIXEL
                                    .waitSeconds(0.4)
                                    .setReversed(true)
                                    .splineToLinearHeading(new Pose2d(35, -20, 0), new Rotation2d(-0.75, 0.75))
                                    .splineToLinearHeading(new Pose2d(-42.5, -14, 0), new Rotation2d(0, 0))//x adevarat este -57.5
                                    .waitSeconds(1)
                                    .build()
                            , new InstantAction(() -> {

                    }),
                            (telemetryPacket) -> {
                                lift.update();
                                if (drive.pose.position.x < -35) {
                                    leftIntakeSv.setPosition(servo_intake_pos + 0.06);
                                    rightIntakeSv.setPosition(servo_intake_pos + 0.06);
                                    banda.setPower(0.8);
                                    intake1.setPower(1);

                                } else {

                                    intake1.setPower(0);
                                }
                                if (drive.pose.position.x > 38) {
                                    lift.goTarget(1600);
                                    banda.setPower(0);
                                } else {
                                    lift.goTarget(0);
                                    ServoUsa.setPosition(servo_usa_inchis);
                                }
                                if (drive.pose.position.x > 48) {
                                    ServoUsa.setPosition(servo_usa_deshis);
                                }
                                if (run.seconds() > 11) {
                                    run.reset();
                                    return false;
                                }
                                telemetry.update();
                                return true;
                            }
                    )
            );
            drive.pose = new Pose2d(-60, -11, Math.toRadians(drive.pose.heading.toDouble()));

            Actions.runBlocking(
                    new ParallelAction(
                            drive.actionBuilder(drive.pose)
                                    .splineToLinearHeading(new Pose2d(15, -10, 0), 0)
                                    .splineToLinearHeading(new Pose2d(51.5, -36, 0), -0.9) // AL DOILEA PIXEL
                                    .waitSeconds(0.4)
                                    .build()
                            , new InstantAction(() -> {

                    }),
                            (telemetryPacket) -> {
                                lift.update();
                                if (drive.pose.position.y < 20)
                                    pixelInit.setPosition(servo_pixel_sus);
                                if (drive.pose.position.x < -35) {
                                    leftIntakeSv.setPosition(servo_intake_pos);
                                    rightIntakeSv.setPosition(servo_intake_pos);
                                    banda.setPower(0.8);
                                    intake1.setPower(1);

                                } else {

                                    intake1.setPower(0);
                                }
                                if (drive.pose.position.x > 35) {
                                    lift.goTarget(1600);
                                    banda.setPower(0);
                                } else {
                                    lift.goTarget(0);
                                    ServoUsa.setPosition(servo_usa_inchis);
                                }
                                if (drive.pose.position.x > 48.5) {
                                    ServoUsa.setPosition(servo_usa_deshis);
                                    return false;
                                }
                                //return false;
                                telemetry.update();
                                return true;
                            }
                    )
            );
            while (total.seconds() < 30) lift.update();


        }

    }

    private void initOpenCV(Telemetry telemetry) {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        OpenCvPipRosuAp OpenCvPipRosuAp = new OpenCvPipRosuAp(telemetry);

        controlHubCam.setPipeline(OpenCvPipRosuAp);
        controlHubCam.openCameraDevice();

        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
}
