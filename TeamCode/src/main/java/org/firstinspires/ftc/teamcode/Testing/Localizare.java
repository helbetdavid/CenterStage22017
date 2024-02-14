package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.RR.tuning.TuningOpModes;

@TeleOp
@Config
public class Localizare extends LinearOpMode {
    double lowSvPos = 0.56;
    double vit = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        DistanceSensor DistSpateSt = hardwareMap.get(DistanceSensor.class, "DistSpateSt");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) DistSpateSt;
        DistanceSensor DistSpateDr = hardwareMap.get(DistanceSensor.class, "DistSpateDr");
        Rev2mDistanceSensor sensorTimeOfFlight1 = (Rev2mDistanceSensor) DistSpateDr;

        DcMotor banda = hardwareMap.get(DcMotorEx.class, "banda");
        DcMotor intake = hardwareMap.get(DcMotorEx.class, "intake");

        // Servos Intake
        Servo rightIntakeSv = hardwareMap.get(Servo.class, "rightIntakeSv");
        Servo leftIntakeSv = hardwareMap.get(Servo.class, "leftIntakeSv");

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
            drive.updatePoseEstimate();

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();







//            if (gamepad1.a) {
//                Actions.runBlocking(drive.actionBuilder(drive.pose)
//                        .strafeTo(new Vector2d(15, 0))
//                        .build()
//                );
//            }
//
//            if (gamepad1.x) {
//                rightIntakeSv.setPosition(lowSvPos);
//                leftIntakeSv.setPosition(lowSvPos);
//                intake.setPower(vit);
//                banda.setPower(vit);
//
//            }
//            if(gamepad1.y){
//                intake.setPower(0);
//                banda.setPower(0);
//            }


            telemetry.addData("pose", drive.pose);
            telemetry.addData("DistSt", DistSpateSt.getDistance(DistanceUnit.CM));
            telemetry.addData("DistDr", DistSpateDr.getDistance(DistanceUnit.CM));
//                telemetry.addData("y", drive.pose.position.y);
//                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();
        }
    }
}


