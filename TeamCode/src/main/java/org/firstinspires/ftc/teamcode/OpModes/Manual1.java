package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static java.lang.Boolean.TRUE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;

@TeleOp
public class Manual1 extends LinearOpMode {
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();


    private PIDFController controller;
    public static double p = 0.0035, i = 0.00002, d = 0.0002;
    public static double f = 0.00005;
    public static int target = 0;
    public static double relatieP = 0.0004;


    public enum RobotState {
        START,
        COLLECTING,
        NEUTRAL,
        SCORRING,
        RETRACTING,
        BEFORE_SCORRING
    }

    RobotState robotState = RobotState.START;


    public static double IntakeLowSvPos = 0.56;
    public static double IntakeMidSvPos = 0.35;
    public static double LiftLowSvPos = 0.243;
    public static double LiftHighSvPos = 0.99;
    public static double minDistnace = 60;
    public static double vitBanda = 1;
    public static int pixel = 0;
    public static int ture = 0;
    public int dom2=1;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime aveon = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        controller = new PIDFController(p, i, d, f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        // Motoare sasiu
        DcMotor leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotor leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotor rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotor rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        //Banda
        DcMotor banda = hardwareMap.get(DcMotorEx.class, "banda");

        // Motoare lift
        DcMotor rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        DcMotor leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");

        // Motoare Intake
        DcMotor intake = hardwareMap.get(DcMotorEx.class, "intake");

        // Servouri Intake
        Servo rightIntakeSv = hardwareMap.get(Servo.class, "rightIntakeSv");
        Servo leftIntakeSv = hardwareMap.get(Servo.class, "leftIntakeSv");

        //Agatat
        CRServo PullupServo = hardwareMap.get(CRServo.class, "PullupServo");
        AnalogInput PullUpEncoder = hardwareMap.get(AnalogInput.class, "PullUpEncoder");

        CRServo Aveon = hardwareMap.get(CRServo.class, "Aveon");


        // Reverse Motors
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Senzori
        DistanceSensor Dist = hardwareMap.get(DistanceSensor.class, "Dist");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) Dist; //CONFIGURATIE!!

//        RevColorSensorV3 colorSensor = (RevColorSensorV3) hardwareMap.colorSensor.get("colorSensor");
//        RevColorSensorV3 colorSensor2 = (RevColorSensorV3) hardwareMap.colorSensor.get("colorSensor");

        PullupServo.setPower(-0.05);
//        Aveon.setPosition(0.1);

        leftIntakeSv.setPosition(0.1);
        rightIntakeSv.setPosition(0.1);

        //Pozitie initiala
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(61, -61.5, 0));


        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive()) {

            controller.setPIDF(p, i, d, f);
            double liftPos = (rightLift.getCurrentPosition() + leftLift.getCurrentPosition()) / 2;
            double pidf = controller.calculate(liftPos, target);
            double motorRelativeError = Math.abs(leftLift.getCurrentPosition() - rightLift.getCurrentPosition()) > 10 ? leftLift.getCurrentPosition() - rightLift.getCurrentPosition() : 0;
            double rightLiftPower = pidf + relatieP * motorRelativeError;
            double leftLiftPower = pidf - relatieP * motorRelativeError;
            double fixer = Math.max(rightLiftPower, Math.max(leftLiftPower, 1));

            rightLift.setPower(rightLiftPower / fixer / 1.25);
            leftLift.setPower(leftLiftPower / fixer / 1.25);


            switch (robotState) {
                case START:
                    intake.setPower(0);
                    banda.setPower(0);
                    target = 0;
                    leftIntakeSv.setPosition(IntakeMidSvPos);
                    rightIntakeSv.setPosition(IntakeMidSvPos);



                    break;
                case COLLECTING:
                    leftIntakeSv.setPosition(IntakeLowSvPos);
                    rightIntakeSv.setPosition(IntakeLowSvPos);

                    intake.setPower(1);
                    banda.setPower(vitBanda);


                    break;
                case NEUTRAL:
                    intake.setPower(0);
                    banda.setPower(0);

                    if (timer.seconds() >= 0.8) {
                        leftIntakeSv.setPosition(IntakeMidSvPos);
                        rightIntakeSv.setPosition(IntakeMidSvPos);
                    }

                    break;
                case SCORRING:
//                    if(Dist.getDistance(DistanceUnit.CM)< 15){
//                        dom2 = 4;
//                    }
//                    else dom2=1;
                    target = 3000;

//                    }

                    break;

                case BEFORE_SCORRING:
                    if(Dist.getDistance(DistanceUnit.CM)< 15){
                        dom2 = 4;
                    }
                    else dom2=1;
                    break;
                case RETRACTING:
                    if (liftPos <= 25) {
                        robotState = RobotState.START;
                    }
                    break;
                default:
                    robotState = RobotState.START;
            }

            if (gamepad2.dpad_down && robotState != RobotState.START) {
                robotState = RobotState.START;
            }

            if (gamepad2.x) {
                robotState = RobotState.COLLECTING;
            }

            if (gamepad2.y) {
                robotState = RobotState.NEUTRAL;

                timer.reset();

            }

            if (gamepad2.b) {
                robotState = RobotState.SCORRING;
                banda.setPower(0);
                timer.reset();
            }

            if (gamepad2.a) {
                target = 0;
                robotState = RobotState.RETRACTING;

            }



            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

//            if (Dist.getDistance(DistanceUnit.CM) < minDistnace && y>0) y = 0;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator/dom2;
            double backLeftPower = (y - x + rx) / denominator/dom2;
            double frontRightPower = (y - x - rx) / denominator/dom2;
            double backRightPower = (y + x - rx) / denominator/dom2;

            //CIPRIAN/VLAD

            double ServVit = gamepad2.left_stick_y;
            PullupServo.setPower(ServVit);



// MIHNEA
            if (gamepad1.right_bumper) {
                leftFront.setPower(frontLeftPower / 4);
                leftRear.setPower(backLeftPower / 4);
                rightFront.setPower(frontRightPower / 4);
                rightRear.setPower(backRightPower / 4);
            } else {
                leftFront.setPower(frontLeftPower);
                leftRear.setPower(backLeftPower);
                rightFront.setPower(frontRightPower);
                rightRear.setPower(backRightPower);
            }


            if (gamepad1.left_bumper){
                aveon.reset();
                Aveon.setPower(1);
            }
            if(aveon.seconds()>1.8){
                Aveon.setPower(0);
            }

            if (gamepad1.x) {
                leftIntakeSv.setPosition(0.5);
                rightIntakeSv.setPosition(0.5);
            }

//            if(gamepad2.left_bumper){
//                banda.setPower(0);
//            }
//            if(gamepad2.right_bumper){
//                banda.setPower(1);
//            }
            if(gamepad2.dpad_up)robotState =RobotState.BEFORE_SCORRING;




            //SPRE BACKDROP

//            if (gamepad1.y) {
//                Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeTo(new Vector2d(-40, -20))
//                                .splineToSplineHeading(new Pose2d(0, 0, 0), 0)
//                                .splineToConstantHeading(new Vector2d(53, 36), 0)
//                                .build()
//                );
//            }

            //SPRE WING

//            if (gamepad1.b) {
//                Actions.runBlocking(drive.actionBuilder(drive.pose)
//                        .strafeTo(new Vector2d(52,-60))
//                        .strafeTo(new Vector2d(13,-16))
//                        .splineTo(new Vector2d(-54,55),-Math.PI*3/2-0.15)
//                        .build()
//                );
//
//            }


            //
//            if ((colorSensor.getDistance(DistanceUnit.CM) + colorSensor2.getDistance(DistanceUnit.CM)) / 2 < 2.65) {
//                pixel += 1;
//                while ((colorSensor.getDistance(DistanceUnit.CM) + colorSensor2.getDistance(DistanceUnit.CM)) / 2 < 2.99) {
//                }
//
//            }


//            if (gamepad1.left_bumper) {
//
//                while (ture < 10) {
//
//                    PullupServo.setPower(0.7);
//                    if (PullUpEncoder.getVoltage() / 3.3 * 360 > 350) {
//                        ture += 1;
//                        while (PullUpEncoder.getVoltage() / 3.3 * 360 > 10) {
//                        }
//                    }
//                }
//                if (ture == 10) PullupServo.setPower(0);
//
//                while (ture >= 10 && ture < 16) {
//
//                    PullupServo.setPower(-0.7);
//                    if (PullUpEncoder.getVoltage() / 3.3 * 360 > 350) {
//                        ture += 1;
//                        while (PullUpEncoder.getVoltage() / 3.3 * 360 > 10) {
//                        }
//                    }
//                }
//                if (ture == 16) PullupServo.setPower(0.05);
//
//            }


//            telemetry.addData("Distance: ", colorSensor.getDistance(DistanceUnit.CM) + " cm");
//            telemetry.addData("Dist", Dist.getDistance(DistanceUnit.CM));

//            telemetry.addData("Distance 2: ", colorSensor2.getDistance(DistanceUnit.CM) + " cm");
//            telemetry.addData("Average", (colorSensor.getDistance(DistanceUnit.CM) + colorSensor2.getDistance(DistanceUnit.CM)) / 2);
            telemetry.addData("Nr pixeli", pixel);
            telemetry.addData("LiftPos ", liftPos);
            telemetry.addData("Target ", target);
            telemetry.addData("Nr pixeli", pixel);
            telemetry.addData("Stadiu Robot", robotState);
            telemetry.addData("Dist fata", Dist.getDistance(DistanceUnit.CM));
//            telemetry.addData("target ", target);
//            telemetry.addData("pos ", liftPos);
//            telemetry.addData("leftPos", leftLift.getCurrentPosition());
//            telemetry.addData("rightPos", rightLift.getCurrentPosition());
//            telemetry.addData("relativeerror", motorRelativeError);
//            telemetry.addData("Distance: ", sensorDistance.getDistance(DistanceUnit.CM) + " cm");
            telemetry.addData("RightFront", rightFront.getCurrentPosition());
            telemetry.addData("RightRear", rightRear.getCurrentPosition());
            telemetry.addData("LeftFront", leftFront.getCurrentPosition());
            telemetry.addData("LeftRear", leftRear.getCurrentPosition());
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.update();

        }
    }
}
