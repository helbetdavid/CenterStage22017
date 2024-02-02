package org.firstinspires.ftc.teamcode.Testing;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCv.OpenCvPipRosu;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
public class camera extends LinearOpMode {

    private static final int CAMERA_WIDTH = 1280;
    private static final int CAMERA_HEIGHT = 720;


    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static volatile OpenCvPipRosu.detectie nou;

    Action pixelToBoardNT,boardToMij,exactBoard;
    @Override
    public void runOpMode() throws InterruptedException {

        //albastru dep
        Pose2d beginPose = new Pose2d(13, 61, -Math.PI / 2);
        Pose2d almostBoard = new Pose2d(48,36,0);
        Pose2d boardMij = new Pose2d(53,36,0);
        Pose2d boardSt = new Pose2d(53,41,0);
        Pose2d boardDr = new Pose2d(53,29,0);
        Pose2d mij = new Pose2d(11,11,0);
        Pose2d stackFront = new Pose2d(-58,11,0);
        Pose2d stackMij = new Pose2d(-58,23.5,0);
        Pose2d stackLast = new Pose2d(-58,35.5,0);
        Pose2d stackPreg = new Pose2d(-40,11,0);



        Lift lift = new Lift();
        DcMotor leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotor leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotor rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotor rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        initOpenCV(telemetry);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
        int v[] = new int[4];
        v[1]=0;
        v[2]=0;
        v[3]=0;
        while(opModeInInit() && !isStopRequested()){
            nou = OpenCvPipRosu.getLocugasit();
            if(nou==OpenCvPipRosu.detectie.Dreapta)v[1]++;
            else if(nou==OpenCvPipRosu.detectie.Stanga)v[2]++;
                else v[3]++;
            telemetry.addData("Detect",nou );
            telemetry.update();
        }
        controlHubCam.stopStreaming();




        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        if(v[1]>v[2] && v[1]>v[3]){
            pixelToBoardNT = drive.actionBuilder(beginPose)
                    .strafeTo(new Vector2d(-49,32))
                    .strafeTo(new Vector2d(-49,36))
                    .strafeToLinearHeading(new Vector2d(48,36),0)
                    .build();
        }
        else if(v[2]>v[1]&& v[2]>v[3]){
            pixelToBoardNT = drive.actionBuilder(beginPose)
                    .strafeTo(new Vector2d(-38,32))
                    .strafeTo(new Vector2d(-38,36))
                    .strafeToLinearHeading(new Vector2d(48,36),0)
                    .build();
        }
        else{
            pixelToBoardNT = drive.actionBuilder(beginPose)
                    .splineTo(new Vector2d(-28, 36), -Math.PI/4)
                    .strafeToLinearHeading(new Vector2d(-41, 58), 0)
                    .strafeTo(new Vector2d(30,58))
//                    .splineTo(new Vector2d(52, 42), 0)
                    .strafeToLinearHeading(new Vector2d(48,36),0)
                    .build();
        }



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            telemetry.addData("Dreapta",v[1] );
            telemetry.addData("Stanga",v[2] );
            telemetry.addData("Mijloc",v[3] );
            if(nou==OpenCvPipRosu.detectie.Dreapta)telemetry.addLine("Dreapta");
                else if(nou== OpenCvPipRosu.detectie.Stanga)telemetry.addLine("Stanga");
                    else telemetry.addLine("Mijloc");
            telemetry.update();


            Actions.runBlocking(
                    new SequentialAction(
                            pixelToBoardNT



//                            drive.actionBuilder( new Pose2d(48, 46, 0)).strafeTo(new Vector2d(53,36)),
//                            new ParallelAction(
//                                lift.goTarget(2000),
//                                    (telemetryPacket) -> {
//
//                                        return false;
//                                    })


                            ));
        }
//        Actions.runBlocking(drive.actionBuilder( new Pose2d(48, 46, 0))
//                .strafeTo(new Vector2d(53,36))
//
//        );
//        controlHubCam.stopStreaming();
        switch (nou){

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


        OpenCvPipRosu openCvPipRosu = new OpenCvPipRosu(telemetry);

        controlHubCam.setPipeline(openCvPipRosu);
        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
}
