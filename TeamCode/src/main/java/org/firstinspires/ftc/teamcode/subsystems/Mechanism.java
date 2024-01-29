package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class Mechanism {

    protected LinearOpMode opMode;


public abstract void init(HardwareMap hwmap);
}
