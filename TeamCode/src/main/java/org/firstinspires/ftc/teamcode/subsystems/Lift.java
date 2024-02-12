package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Lift extends Mechanism{

    public static double p = 0.0035, i = 0.00002, d = 0.0002;
    public static double f = 0.00005;
    public static int target = 0;
    DcMotorEx rightLift, leftLift;
    public static double relatieP = 0.0004  ;
    private final PIDFController controller=new PIDFController(p, i, d, f);;
    Telemetry tele;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry telemetry;
    public boolean isReached=false;
    double liftPos, pidf, motorRelativeError, rightLiftPower, leftLiftPower, fixer;
    @Override
    public void init(HardwareMap hwmap) {
        rightLift = hwmap.get(DcMotorEx.class, "rightLift");
        leftLift = hwmap.get(DcMotorEx.class, "leftLift");

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        controller.setPIDF(p, i, d, f);
    }

    public void update(){
        liftPos = (rightLift.getCurrentPosition() + leftLift.getCurrentPosition()) / 2.0;
        pidf = controller.calculate(liftPos, target);
        motorRelativeError = Math.abs(leftLift.getCurrentPosition() - rightLift.getCurrentPosition()) > 10 ? leftLift.getCurrentPosition() - rightLift.getCurrentPosition() : 0;
        rightLiftPower = pidf + relatieP * motorRelativeError;
        leftLiftPower = pidf - relatieP * motorRelativeError;
        fixer = Math.max(rightLiftPower, Math.max(leftLiftPower, 1));


        if(liftPos<=target-30){
            isReached=true;
        }
        rightLift.setPower(rightLiftPower / fixer / 1.5);
        leftLift.setPower(leftLiftPower / fixer / 1.5);

    }
    public int getPositionLeft(){
        return leftLift.getCurrentPosition();
    }

    public int getPositionRight(){
        return rightLift.getCurrentPosition();
    }
    public void initTele(Telemetry tele){
        telemetry =new MultipleTelemetry(tele,dashboard.getTelemetry());
    }
    public void goTarget(int x){
        target=x;
        return;
    }

    public void liftLow(){
        target  = -30;
        return;
    }

    public void liftMid(){
        target = 1500;
        return;
    }

    public void liftHigh(){
        target = 2200;
        return;
    }
}
