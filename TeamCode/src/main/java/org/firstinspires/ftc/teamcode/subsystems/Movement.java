package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Movement extends Mechanism{
    DcMotor leftFront, leftRear, rightRear, rightFront;
    Telemetry tele;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry telemetry;


    @Override
    public void init(HardwareMap hwmap) {

         leftFront = hwmap.get(DcMotorEx.class, "leftFront");
         leftRear = hwmap.get(DcMotorEx.class, "leftRear");
         rightRear = hwmap.get(DcMotorEx.class, "rightRear");
         rightFront = hwmap.get(DcMotorEx.class, "rightFront");

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void forward(double viteza) {
        leftFront.setPower(viteza);
        leftRear.setPower(viteza);
        rightFront.setPower(viteza);
        rightRear.setPower(viteza);

    }

    public void turn(double v, double y){
        leftFront.setPower(v);
        leftRear.setPower(v);
        rightFront.setPower(-v);
        rightRear.setPower(-v);



    }
}
