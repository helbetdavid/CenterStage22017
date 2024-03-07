package org.firstinspires.ftc.teamcode.OpenCv;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpenCvPipAlbastruNou extends OpenCvPipeline {

    public enum GamePropPosition
    {
        STANGA,
        MIJLOC,
        DREAPTA
    }
    static final Scalar Black = new Scalar(0, 0, 0);
    static final Scalar Green = new Scalar(0, 255, 0);

    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(109,98);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(440,525);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(950,550);
    public static int REGION_WIDTH = 125;
    public static int REGION_HEIGHT = 125;

    public static int H=105;
    public static int S=140;
    public static int V=90;
    public static int HH=120;
    public static int SH=255;
    public static int VH=255;

    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    Mat region1_Cb, region2_Cb, region3_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    int avg1, avg2, avg3;

    private volatile GamePropPosition position = GamePropPosition.STANGA;

    void 
}
