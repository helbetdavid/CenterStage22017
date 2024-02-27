package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCv.OpenCvPipAlbastru;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
@Config
public class camera extends LinearOpMode {
    OpenCvWebcam webcam;
    OpenCvPipAlbastru pipeline;
    private static final int CAMERA_WIDTH = 1280;
    private static final int CAMERA_HEIGHT = 720;
    @Override
    public void runOpMode(){

        initOpenCV();
        while(opModeInInit() && !isStopRequested()){

            telemetry.addData("loc",pipeline.getAnalysis());
            telemetry.update();

            sleep(50);
        }
        while(opModeIsActive()){

        }
    }

    private void initOpenCV() {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        OpenCvPipAlbastru openCvPipAlbastru = new OpenCvPipAlbastru();

        webcam.setPipeline(openCvPipAlbastru);
        webcam.openCameraDevice();

        webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
}
