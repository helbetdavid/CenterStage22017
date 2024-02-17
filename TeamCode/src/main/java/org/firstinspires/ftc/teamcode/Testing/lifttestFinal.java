package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp

public class lifttestFinal extends LinearOpMode {
    private PIDFController controller;

    public static double inmServo = 0.5;
    public static double inmGlisiere = 0.5;

    public static double p = 0.0061, i = 0.00012 , d = 0.00005;
    public static double f = 0.00003;
    public static int target = 0;
    public static double relatieP = 0.0053;


    @Override
    public void runOpMode() throws InterruptedException {

        controller = new PIDFController(p, i, d,f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Declaram Motoare
        DcMotor rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        DcMotor leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        DcMotor banda = hardwareMap.get(DcMotorEx.class, "banda");
        DcMotor intake = hardwareMap.get(DcMotorEx.class, "intake");

        //Schimbam Directie
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);

        CRServo Usa = hardwareMap.get(CRServo.class, "Usa");

        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {

            controller.setPIDF(p, i, d,f);
            double liftPos = (rightLift.getCurrentPosition() + leftLift.getCurrentPosition()) / 2.0;
            double pidf = controller.calculate(liftPos, target);
            double motorRelativeError = Math.abs(leftLift.getCurrentPosition()-rightLift.getCurrentPosition())>10?leftLift.getCurrentPosition()-rightLift.getCurrentPosition():0;

            double rightLiftPower = pidf+relatieP*motorRelativeError;
            double leftLiftPower = pidf-relatieP*motorRelativeError;
            double denom = Math.max(rightLiftPower,Math.max(leftLiftPower,1));

            rightLift.setPower(rightLiftPower/denom * inmGlisiere);
            leftLift.setPower(leftLiftPower/denom* inmGlisiere);
            if(Math.abs(target-liftPos) > 20) {
                if (leftLiftPower > 0)
                    Usa.setPower(1);
                else if (leftLiftPower < 0)
                    Usa.setPower(-1);
                else Usa.setPower(0);
            }

            telemetry.addData("target ", target);
            telemetry.addData("pos ", liftPos);
            telemetry.addData("leftPos", leftLift.getCurrentPosition());
            telemetry.addData("rightPos", rightLift.getCurrentPosition());
            telemetry.addData("banda",banda.getCurrentPosition());
            telemetry.addData("intake", intake.getCurrentPosition());
            telemetry.addData("relativeerror",motorRelativeError);
            telemetry.update();

        }

    }
}
