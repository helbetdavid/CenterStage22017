package org.firstinspires.ftc.teamcode.Testing;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCv.OpenCvPipRosu;
import org.firstinspires.ftc.teamcode.OpenCv.OpenCvPipAlbastruAp;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
public class Distractie extends LinearOpMode {

    private static final int CAMERA_WIDTH = 1280;
    private static final int CAMERA_HEIGHT = 720;


    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK



    @Override
    public void runOpMode() throws InterruptedException {

        initOpenCV(telemetry);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            telemetry.addData("Detect", OpenCvPipAlbastruAp.getAnalysis());
            telemetry.update();
        }
        controlHubCam.stopStreaming();
    }

    private void initOpenCV(Telemetry telemetry) {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        OpenCvPipAlbastruAp openCvPipAlbastruAp = new OpenCvPipAlbastruAp(telemetry);

        controlHubCam.setPipeline(openCvPipAlbastruAp);
        controlHubCam.openCameraDevice();

        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
}
