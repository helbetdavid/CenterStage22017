package org.firstinspires.ftc.teamcode.OpenCv;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpenCvPipAlbastruAp extends OpenCvPipeline {

    Telemetry telemetry;
    private static volatile detectie locugasit;
    public OpenCvPipAlbastruAp(Telemetry tele){
        this.telemetry = tele;
    }
    public enum detectie{
        Stanga,
        Dreapta,
        Mijloc
    }



    static final Scalar Black = new Scalar(0, 0, 0);
    static final Scalar Green = new Scalar(0, 255, 0);

    //Colturile din stanga sus ale dreptunghiurilor
    public static Point DREPTUNGHI_2_COLT_STANGA_SUS = new Point(700, 275);
    public static Point DREPTUNGHI_3_COLT_STANGA_SUS = new Point(120, 250);
    public static int REGION_WIDTH = 125;
    public static int REGION_HEIGHT = 125;

    public static int H=110;
    public static int S=180;
    public static int V=85;
    public static int HH=120;
    public static int SH=255;
    public static int VH=255;

    // Anchor point definitions

    public static Point DREPTUNGHI_2_COLT_DREAPTA_JOS = new Point(
            DREPTUNGHI_2_COLT_STANGA_SUS.x + REGION_WIDTH,
            DREPTUNGHI_2_COLT_STANGA_SUS.y + REGION_HEIGHT);
    public static Point DREPTUNGHI_3_COLT_DREAPTA_JOS = new Point(
            DREPTUNGHI_3_COLT_STANGA_SUS.x + REGION_WIDTH+40,
            DREPTUNGHI_3_COLT_STANGA_SUS.y + REGION_HEIGHT+40);

    public static double threshold = 200000;

    public void draw(detectie pozitiecurenta, Mat input, Mat mat, Point sus, Point jos){
        Scalar sum = Core.sumElems(mat);

        if(sum.val[0] > threshold){
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    sus, // First point which defines the rectangle
                    jos, // Second point which defines the rectangle
                    Green, // The color the rectangle is drawn in
                    3); // Thickness of the rectangle lines
            locugasit = pozitiecurenta;
        }else{
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    sus, // First point which defines the rectangle
                    jos, // Second point which defines the rectangle
                    Black, // The color the rectangle is drawn in
                    3); // Thickness of the rectangle lines
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        locugasit=detectie.Dreapta;
        Mat output = input.clone();

        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);


        Scalar lowHSV = new Scalar(H, S, V);
        Scalar highHSV = new Scalar(HH, SH, VH);




        Core.inRange(input,lowHSV,highHSV,input);

        draw(detectie.Mijloc, output, input.submat(new Rect(DREPTUNGHI_2_COLT_STANGA_SUS, DREPTUNGHI_2_COLT_DREAPTA_JOS)), DREPTUNGHI_2_COLT_STANGA_SUS, DREPTUNGHI_2_COLT_DREAPTA_JOS);
        draw(detectie.Stanga, output, input.submat(new Rect(DREPTUNGHI_3_COLT_STANGA_SUS, DREPTUNGHI_3_COLT_DREAPTA_JOS)), DREPTUNGHI_3_COLT_STANGA_SUS, DREPTUNGHI_3_COLT_DREAPTA_JOS);




        telemetry.addData("pozitie", locugasit);
        telemetry.update();
        return output ;
    }
    public static detectie getLocugasit() {
        return locugasit;
    }
}