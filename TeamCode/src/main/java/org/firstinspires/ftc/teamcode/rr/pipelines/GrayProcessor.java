package org.firstinspires.ftc.teamcode.rr.pipelines;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
//import org.firstinspires.ftc.
//import OpModes.FTCWiresAutoVisionOpenCV;

public class GrayProcessor implements VisionProcessor {
//   FTCWiresAutoVisionOpenCV.CameraSelectedAroundMid selectionAroundMid = FTCWiresAutoVisionOpenCV.CameraSelectedAroundMid.NONE;

    public static double fx = 578.272;
    public static double fy = 578.272;
    public static double cx = 402.145;
    public static double cy = 221.506;
    Mat submat = new Mat();
    Mat hsvMat = new Mat();

    public double satRectLeftOfCameraMid, satRectRightOfCameraMid;
    public double satRectNone = 40.0;
    public Rect rectLeftOfCameraMid, rectRightOfCameraMid;
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        rectLeftOfCameraMid = new Rect(10, 40, 150, 240);
        rectRightOfCameraMid = new Rect(160, 40, 470, 160);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        satRectLeftOfCameraMid = getAvgSaturation(hsvMat, rectLeftOfCameraMid);
        satRectRightOfCameraMid = getAvgSaturation(hsvMat, rectRightOfCameraMid);

//        if ((satRectLeftOfCameraMid > satRectRightOfCameraMid) && (satRectLeftOfCameraMid > satRectNone)) {
//            return FTCWiresAutoVisionOpenCV.CameraSelectedAroundMid.LEFT_OF_CAMERA_MID;
//        } else if ((satRectRightOfCameraMid > satRectLeftOfCameraMid) && (satRectRightOfCameraMid > satRectNone)) {
//            return FTCWiresAutoVisionOpenCV.CameraSelectedAroundMid.RIGHT_OF_CAMERA_MID;
//        }
//        return FTCWiresAutoVisionOpenCV.CameraSelectedAroundMid.NONE;
        return null;
    }
    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }
    protected double getAvgSaturation(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonSelectedPaint = new Paint(selectedPaint);
        nonSelectedPaint.setColor(Color.GREEN);

        android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeftOfCameraMid, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectRightOfCameraMid, scaleBmpPxToCanvasPx);

        canvas.drawRect(drawRectangleLeft, selectedPaint);
        canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
    }
}