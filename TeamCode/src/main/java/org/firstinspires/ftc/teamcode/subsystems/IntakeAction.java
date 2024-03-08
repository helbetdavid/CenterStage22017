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

    public static double p = 0.0052, i = 0.000125 , d = 0.000062;
    public static double f = 0.000034;
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
    private Servo usa = null;
    public static double servo_usa_inchis= 0.5;
    public static double servo_usa_deshis= 0.2;
    private DcMotorEx banda = null;
    private Servo rightIntakeSv = null;
    private Servo pixelInit = null;
    public static double servo_pixel_sus= 0.4;
    public static double servo_pixel_jos = 0.91;
    ElapsedTime timer = new ElapsedTime();



    public IntakeAction(HardwareMap hwmap){

        intake = hwmap.get(DcMotorEx.class, "intake");
        rightIntakeSv = hwmap.get(Servo.class, "rightIntakeSv");
        leftIntakeSv = hwmap.get(Servo.class, "leftIntakeSv");
        usa = hwmap.get(Servo.class, "Usa");

        banda =hwmap.get(DcMotorEx.class, "banda");

        rightLift = hwmap.get(DcMotorEx.class, "rightLift");
        leftLift = hwmap.get(DcMotorEx.class, "leftLift");
        pixelInit = hwmap.get(Servo.class, "PixelStartSv");

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
            timer.reset();
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

            return !(liftPos >= target - 50) && !(timer.seconds() > 3);
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
    public class SetUsaOpen implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            usa.setPosition(servo_usa_deshis);
            return false;
        }
    }
    public class SetUsaClose implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            usa.setPosition(servo_usa_inchis);
            return false;
        }
    }
    public class SetLatSus implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            pixelInit.setPosition(servo_pixel_sus);
            return false;
        }
    }
    public class SetLatJos implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            pixelInit.setPosition(servo_pixel_sus);
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
    public Action setUsaOpen(){return new SetUsaOpen();}
    public Action setUsaClose(){return new SetUsaClose();}
    public Action setLatSus(){return new SetLatSus();}
    public Action setLatJos(){return new SetLatJos();}





}
