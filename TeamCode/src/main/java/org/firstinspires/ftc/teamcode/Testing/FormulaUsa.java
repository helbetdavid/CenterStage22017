package org.firstinspires.ftc.teamcode.Testing;

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

import org.firstinspires.ftc.robotcore.external.Telemetry;
@TeleOp
public class FormulaUsa extends LinearOpMode {
    public double rotatii = 0;
    public double rotatii2 = 0;
    private PIDFController controller;
    public static double p = 0.0035, i = 0.00002, d = 0.0002;
    public static double f = 0.00005;
    public static int target = 0;
    public static double relatieP = 0.0004;

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

            rightLift.setPower(rightLiftPower / fixer / 1.25);
            leftLift.setPower(leftLiftPower / fixer / 1.25);

//            target = 1500;
            if(gamepad1.a){
                Usa.setPower(0.2);
                if(UsaEncoder.getVoltage() / 3.3 * 360 >= 350){
                    rotatii++;
                    while(UsaEncoder.getVoltage() / 3.3 * 360 > 10){}
                }

            } else if (gamepad1.x) {
                Usa.setPower(-0.2);
                if(UsaEncoder.getVoltage() / 3.3 * 360 <= 10){
                    rotatii2--;
                    while(UsaEncoder.getVoltage() / 3.3 * 360 <350){}
                }

            } else if(gamepad1.b) Usa.setPower(0);








            telemetry.addData("Pos", liftPos );
            telemetry.addData("Tickuri Servo", UsaEncoder.getVoltage() / 3.3 * 360 + Math.abs(rotatii -rotatii2)* 360);
            telemetry.addData("Tickuri Servo 2", -UsaEncoder.getVoltage() / 3.3 * 360 + Math.abs(rotatii -rotatii2) * 360);
            telemetry.addData("Voltaj ", UsaEncoder.getVoltage());
            telemetry.update();


        }


    }
}
