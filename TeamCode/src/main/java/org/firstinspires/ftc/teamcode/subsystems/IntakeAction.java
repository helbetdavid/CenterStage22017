package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeAction {

    public static double p = 0.0052, i = 0.000135 , d = 0.000055;
    public static double f = 0.000035;
    public static int target = 0;
    public static double relatieP = 0.0062;
    DcMotorEx rightLift, leftLift;
    private final PIDFController controller=new PIDFController(p, i, d, f);

    Telemetry tele;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry telemetry;
    public boolean isReached=false;
    double liftPos, pidf, motorRelativeError, rightLiftPower, leftLiftPower, fixer;
    private DcMotorEx intake  = null;
    private Servo leftIntakeSv = null;
    private DcMotorEx banda = null;
    private Servo rightIntakeSv = null;
    ElapsedTime timer = new ElapsedTime();



    public IntakeAction(HardwareMap hwmap){

        intake = hwmap.get(DcMotorEx.class, "intake");
        rightIntakeSv = hwmap.get(Servo.class, "rightIntakeSv");
        leftIntakeSv = hwmap.get(Servo.class, "leftIntakeSv");

        banda =hwmap.get(DcMotorEx.class, "banda");

        rightLift = hwmap.get(DcMotorEx.class, "rightLift");
        leftLift = hwmap.get(DcMotorEx.class, "leftLift");

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        controller.setPIDF(p, i, d, f);




    }

    public class IntakeStart implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setPower(1);
            banda.setPower(1);
            return false;
        }
    }

    public class IntakeStop implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setPower(0);
            banda.setPower(0);
            return false;
        }
    }

    public class Wait implements Action{


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            timer.reset();
            while(timer.seconds()< 5){

            }


            return false;
        }
    }

    public class SetPosition implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            leftIntakeSv.setPosition(0.4);
            rightIntakeSv.setPosition(0.4);
            return false;
        }
    }


    public class GotoTarget implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            liftPos = (rightLift.getCurrentPosition() + leftLift.getCurrentPosition()) / 2.0;
            pidf = controller.calculate(liftPos, target);
            motorRelativeError = Math.abs(leftLift.getCurrentPosition() - rightLift.getCurrentPosition()) > 10 ? leftLift.getCurrentPosition() - rightLift.getCurrentPosition() : 0;
            rightLiftPower = pidf + relatieP * motorRelativeError;
            leftLiftPower = pidf - relatieP * motorRelativeError;
            fixer = Math.max(rightLiftPower, Math.max(leftLiftPower, 1));


            if(liftPos>=target-50){
                isReached=true;
            }
            rightLift.setPower(rightLiftPower / fixer );
            leftLift.setPower(leftLiftPower / fixer );


            return false;
        }

    }

    public class SetTargetLow implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            target = 800;
            return false;
        }
    }

    public class SetTargetMid implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            target = 1200;
            return false;
        }
    }

    public class SetTargetHigh implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            target = 2000;
            return false;
        }
    }

    public Action Start(){
        return new IntakeStart();
    }

    public Action Stop(){
        return new IntakeStop();
    }

    public Action Wait() { return new Wait(); }

    public Action setPosition(){ return new SetPosition();}

    public Action gotToTarget(){ return new GotoTarget();}

    public Action setTargetLow(){return new SetTargetLow();}
    public Action setTargetMid(){return new SetTargetMid();}
    public Action setTargetMigh(){return new SetTargetHigh();}





}
