package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*  Pasi ca sa testezi numaratorul de pixeli:

 *  1. Testeaza sa vezi ce-ti da in telemetry "intake.getCurrent(CurrentUnit.AMPS)
 *      si vezi ce valoare iti da cand motorul este "blocat" (are un pixel)

 *  2. Schimba currentThreshold din 0 in valoarea pe care ai vazut-o in telemetry (aia la motorul blocat)

 *  3. Vezi daca iti creste corect numarul de pixeli (variabila numita "pixeli")

 *  4. Vezi cum merge si cu banda

 *  Optional, daca vrei sa te si misti cu robotu in timp ce ia pixelii, doar da de manetute:)
 *  P.S. S-ar putea sa te misti putin mai greu cu intake-ul dat jos, asa ca ai si un buton care sa-l ridice.
 *       Sa-ti fie mai usor (tre sa tii apasat pe buton).

    P.S.2  Daca nu functioneaza cum ar trebui, mai am o varianta :) (doar spune-mi)
 *
 *     */

@TeleOp
@Config
public class NrPixeli extends LinearOpMode {
    DcMotorEx intake, banda;
    Servo leftIntakeSv, rightIntakeSv;

    //TO DO 2 :)
    static double currentThreshold = 0;

    static double downSvPos = 0.5;
    static double upSvPos = 0.1;

    static double pwr = 1;
    static double pwr2 = 1;
    static double pwrNull = 0;
    double reversePwr;

    double pixeli = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        //Declarare motoare sasiu
        DcMotor leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotor leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotor rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotor rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        //Comportament la putere 0
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Declarare motoare banda si intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        banda = hardwareMap.get(DcMotorEx.class, "banda");

        //Declarare servo-uri intake
        leftIntakeSv = hardwareMap.get(Servo.class, "leftIntakeSv");
        rightIntakeSv = hardwareMap.get(Servo.class, "rightIntakeSv");

        //Setare pozitie initiala servo-uri intake
        leftIntakeSv.setPosition(upSvPos);
        rightIntakeSv.setPosition(upSvPos);

        waitForStart();


        while (opModeIsActive() && !isStopRequested()) {


            //TO DO 1 (dupa ce ai terminat, comentezi codul asta)

            intake.setPower(pwr);
            if(gamepad1.x)  // gamepad1.x inseamna patrat pe controllerul nostru
                pwr = 0;

            if(gamepad1.y) // gamepad1.y inseamna triunghi pe controllerul nostru
                pwr = 1;

            telemetry.addData("Amps", intake.getCurrent(CurrentUnit.AMPS));
            telemetry.update();



            //TO DO 3 (dupa ce ai terminat, comentezi codul asta)


//            intake.setPower(pwr);
//
//            if (intake.getCurrent(CurrentUnit.AMPS) > currentThreshold) {
//                pixeli++;
//            }
//
//            if(pixeli == 2){
//                pwr =0;
//            }
//
//            if(gamepad1.y) { // gamepad1.y inseamna triunghi pe controllerul nostru
//                pixeli = 0;
//                pwr = 1;
//
//            }

//            if(gamepad1.x)  // gamepad1.x inseamna patrat pe controllerul nostru
//                pwr = 0;  // in caz ca vrei sa opresti intake din oarecare motiv

//            telemetry.addData("pixeli", pixeli);
//            telemetry.addData("Amps",intake.getCurrent(CurrentUnit.AMPS));
//            telemetry.update();



            //TO DO 4

//            leftIntakeSv.setPosition(downSvPos);
//            rightIntakeSv.setPosition(downSvPos);
//
//            banda.setPower(pwr2);
//            intake.setPower(pwr);
//
//            if (intake.getCurrent(CurrentUnit.AMPS) > currentThreshold) {
//                pixeli++;
//            }
//
//            if(pixeli == 2){
//                pwr =0;
//                pwr2 = 0;
//                downSvPos = 0.1;
//
//            }
//
//            if(gamepad1.y) {
//                pixeli = 0;
//                pwr = 1;
//                pwr2 = 1;
//                downSvPos = 0.5;
//            }

//            if(gamepad1.x) { // gamepad1.x inseamna patrat pe controllerul nostru
//                pwr = 0;
//                pwr2 =0;
//            }


//            if(gamepad1.right_bumper){
//                leftIntakeSv.setPosition(0.4);
//                rightIntakeSv.setPosition(0.4);
//            }
//
//            double y = -gamepad1.left_stick_y;
//            double x = gamepad1.left_stick_x * 1.1;
//            double rx = gamepad1.right_stick_x;
//
//
//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//            double frontLeftPower = (y + x + rx) / denominator;
//            double backLeftPower = (y - x + rx) / denominator;
//            double frontRightPower = (y - x - rx) / denominator;
//            double backRightPower = (y + x - rx) / denominator;
//
//            leftFront.setPower(frontLeftPower);
//            leftRear.setPower(backLeftPower);
//            rightFront.setPower(frontRightPower);
//            rightRear.setPower(backRightPower);

//            telemetry.addData("pixeli", pixeli);
//            telemetry.addData("Amps",intake.getCurrent(CurrentUnit.AMPS));
//            telemetry.update();




        }
    }
}
