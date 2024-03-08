package org.firstinspires.ftc.teamcode.Autonoms;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCv.OpenCvPipRosu;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeAction;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Movement;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class RosuAp extends LinearOpMode {

    public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
    Pose2d beginPose = new Pose2d(13, -61, Math.PI / 2);
    IMU imu;


    public static double servo_pixel_sus = 0.4;
    public static double servo_pixel_jos = 0.96;
    public static double servo_intake_pos = 0.44;
    public static double servo_usa_inchis = 0.5;
    public static double servo_usa_deshis = 0.3;
    public static double heading;
    public static int numaratoare = 0;

    public static boolean dat_dru = false;
    private static final int CAMERA_WIDTH = 1280;
    private static final int CAMERA_HEIGHT = 720;

    public enum StackPixel {
        pixelStackFront,
        pixelStackMid,
        pixelStackFar
    }

    public StackPixel stackPixel;
    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static volatile OpenCvPipRosu.detectie nou;

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

        Servo rightIntakeSv = hardwareMap.get(Servo.class, "rightIntakeSv");
        Servo leftIntakeSv = hardwareMap.get(Servo.class, "leftIntakeSv");
        Servo pixelInit = hardwareMap.get(Servo.class, "PixelStartSv");
        Servo ServoUsa = hardwareMap.get(Servo.class, "Usa");


        IntakeAction intakenou = new IntakeAction(hardwareMap);
        Lift lift = new Lift();
        Movement movement = new Movement();
        Intake intake = new Intake();
        DcMotor leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotor leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotor rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotor rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        lift.init(hardwareMap);
        movement.init(hardwareMap);
        intake.init(hardwareMap);

        Vector2d caz = null;
        Vector2d board = null;


        initOpenCV(telemetry);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        int v[] = new int[4];
        v[1] = 0;
        v[2] = 0;
        v[3] = 0;

        ElapsedTime pixel = new ElapsedTime();
        ElapsedTime pixel2 = new ElapsedTime();
        pixel.startTime();
        pixel2.startTime();
        while (opModeInInit() && !isStopRequested()) {
            if(pixel.seconds()>1) {
                nou = OpenCvPipRosu.getAnalysis();
                if (nou == OpenCvPipRosu.detectie.Dreapta) v[1]++;
                else if (nou == OpenCvPipRosu.detectie.Stanga) v[2]++;
                else v[3]++;
                pixel.reset();
            }
            if(pixel2.seconds() > 5){
                v[1]=0;
                v[2]=0;
                v[3]=0;
                pixel2.reset();
            }
            pixelInit.setPosition(servo_pixel_jos);
            intake.intakePos(0.1);
            ServoUsa.setPosition(servo_usa_inchis);
            telemetry.addData("Detect", nou);
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
            if (nou == OpenCvPipRosu.detectie.Dreapta) telemetry.addLine("Dreapta");
            else if (nou == OpenCvPipRosu.detectie.Stanga) telemetry.addLine("Stanga");
            else telemetry.addLine("Mijloc");
            telemetry.update();
//

            if(v[3]>v[1] && v[3]>v[2]){ //Mijloc
                caz = new Vector2d(17, -31);
                board = new Vector2d(56.5, -36);
                heading=0;
            }
            else if(v[2]>v[1] && v[2]>v[3]){ //Stanga
                caz = new Vector2d(10, -30.5);
                board = new Vector2d(56.5, -28.5);
                heading = Math.PI/2;
            }
            else{ //Dreapta
                caz = new Vector2d(25, -40);
                board = new Vector2d(56.5, -41);
                heading = 0;
            }

            Actions.runBlocking(
                    new ParallelAction(
                            drive.actionBuilder(beginPose)
                                    .strafeToLinearHeading(caz, heading)// PRIMUL PIXEL
                                    .stopAndAdd(intakenou.setLatSus())
                                    .strafeToLinearHeading(board, 0)
                                    .waitSeconds(0.5)
                                    .stopAndAdd(intakenou.setUsaOpen())// AL DOILEA PIXEL
                                    .waitSeconds(0.5)
                                    .stopAndAdd(intakenou.setUsaClose())
                                    .strafeToLinearHeading(new Vector2d(34, -52), 0)
                                    .strafeToLinearHeading(new Vector2d(61, -59), 0)
                                    .build()
                            ,
                            (telemetryPacket) -> {
                                lift.update();
                                if(total.seconds()>3.5)lift.goTarget(1100);
                                if(total.seconds()>7)lift.goTarget(0);
                                if(total.seconds()>15)return false;
                                return true;
                            }
                    )
            );

        }


    }

    private void initOpenCV(Telemetry telemetry) {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        OpenCvPipRosu OpenCvPipRosu = new OpenCvPipRosu(telemetry);

        controlHubCam.setPipeline(OpenCvPipRosu);
        controlHubCam.openCameraDevice();

        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
}
