package org.firstinspires.ftc.teamcode.Testing;

import static java.sql.Types.NULL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.ejml.equation.IntegerSequence;
import org.firstinspires.ftc.robotcore.external.Telemetry;
@TeleOp
public class FormulaUsa extends LinearOpMode {
    double ForTelemetry;
    double CurrentbucEncoder;
    double LastBucEncoder;
    public double rotatii = 0;
    public double rotatii2 = 0;
    private PIDFController controller;
    public static double p = 0.003, i = 0.000077, d = 0.000065;
    public static double f = 0.000002;
    public static int target = 0;
    public static double relatieP = 0.00275;

    @Override
    public void runOpMode() throws InterruptedException {
        CRServo Usa = hardwareMap.get(CRServo.class, "Usa");
        AnalogInput UsaEncoder = hardwareMap.get(AnalogInput.class, "UsaEncoder");
        DcMotor rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        DcMotor leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);


        controller = new PIDFController(p, i, d, f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());




        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            controller.setPIDF(p, i, d, f);
            double liftPos = (rightLift.getCurrentPosition() + leftLift.getCurrentPosition()) / 2;
            double pidf = controller.calculate(liftPos, target);
            double motorRelativeError = Math.abs(leftLift.getCurrentPosition() - rightLift.getCurrentPosition()) > 10 ? leftLift.getCurrentPosition() - rightLift.getCurrentPosition() : 0;
            double rightLiftPower = pidf + relatieP * motorRelativeError;
            double leftLiftPower = pidf - relatieP * motorRelativeError;
            double fixer = Math.max(rightLiftPower, Math.max(leftLiftPower, 1));






            rightLift.setPower(rightLiftPower / fixer / 1.250);
            leftLift.setPower(leftLiftPower / fixer / 1.25);


            while(gamepad1.a){
                CurrentbucEncoder= UsaEncoder.getVoltage() / 3.3 * 360;

//                CurrentbucEncoder= UsaEncoder.getVoltage() / 3.3 * 360;

                Usa.setPower(0.2);
                ForTelemetry = CurrentbucEncoder;
                if(LastBucEncoder > CurrentbucEncoder) rotatii++;


                LastBucEncoder = CurrentbucEncoder;

                telemetry.addData("Tickuri Servo",   ForTelemetry + (int)rotatii/88* 360);
                telemetry.addData("rotatii", (int)rotatii/88);
                telemetry.addData("Servo", UsaEncoder.getVoltage() / 3.3 * 360);
                telemetry.update();

            }
            Usa.setPower(0);


            while (gamepad1.x) {
                CurrentbucEncoder= UsaEncoder.getVoltage() / 3.3 * 360;



//                target = 1500;
                Usa.setPower(-0.2);
                ForTelemetry = CurrentbucEncoder;
                if(LastBucEncoder < CurrentbucEncoder) rotatii--;
                LastBucEncoder = CurrentbucEncoder;

                telemetry.addData("Tickuri Servo",   ForTelemetry + ((int)rotatii/88 -1)* 360);
                telemetry.addData("rotatii", (int)rotatii/88);
                telemetry.addData("Servo", UsaEncoder.getVoltage() / 3.3 * 360);
                telemetry.update();

//                telemetry.addData("Tickuri Servo", CurrentbucEncoder + rotatii* 360);
//                telemetry.update();
//                if(UsaEncoder.getVoltage() / 3.3 * 360 <= 10){
//                    rotatii--;
//                    while(UsaEncoder.getVoltage() / 3.3 * 360 <350){}
//                }




            }
            Usa.setPower(0);









//            telemetry.addData("Tickuri Servo",   ForTelemetry + rotatii* 360);
//            telemetry.addData("Servo", UsaEncoder.getVoltage() / 3.3 * 360);
//            telemetry.addData("ForTelemetry", ForTelemetry);
//            telemetry.addData("rotatii", rotatii);
//            telemetry.update();
//            telemetry.addData("Pos", liftPos );


        }


    }
}