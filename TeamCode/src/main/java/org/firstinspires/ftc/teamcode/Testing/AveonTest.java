package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
@Config
public class AveonTest extends LinearOpMode {
    public  static double vit = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo aveon = hardwareMap.get(Servo.class, "leftIntakeSv");

        waitForStart();
        while(opModeIsActive()){
            aveon.setPosition(vit);
        }
    }
}
