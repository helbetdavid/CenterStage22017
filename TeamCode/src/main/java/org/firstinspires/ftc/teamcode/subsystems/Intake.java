package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends Mechanism{

    DcMotor intake, banda;
    Servo rightIntakeSv, leftIntakeSv;
    @Override
    public void init(HardwareMap hwmap) {
         intake = hwmap.get(DcMotorEx.class, "intake");
         rightIntakeSv = hwmap.get(Servo.class, "rightIntakeSv");
         leftIntakeSv = hwmap.get(Servo.class, "leftIntakeSv");

        banda =hwmap.get(DcMotorEx.class, "banda");

    }

    public void pwrIntake(double vit){
        intake.setPower(vit);
    }

    public void intakePos(double pos){
        leftIntakeSv.setPosition(pos);
        rightIntakeSv.setPosition(pos);
    }

    public void pwrBanda(double vit){
        banda.setPower(vit);

    }
}
