package OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;

@Autonomous
@Config
public class Testing extends LinearOpMode {
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();


    private PIDFController controller;
    public static double p = 0.00377, i = 0.00002, d = 0.0002;
    public static double f = 0.00005;
    public static int target = 0;
    public static double relatieP = 0.0006;


    public enum RobotState {
        START,
        COLLECTING,
        NEUTRAL,
        SCORRING,
        RETRACTING,
        IDLE
    }

    RobotState robotState = RobotState.START;
    public static int bolcase=0;


    public static double IntakeLowSvPos = 0.56;
    public static double IntakeMidSvPos = 0.35;
    public static double LiftLowSvPos = 0.243;
    public static double LiftHighSvPos = 0.99;
    public static double minDistnace = 60;
    public static double vitBanda = 1;
    public static int pixel = 0;
    public static int ture = 0;

    ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        controller = new PIDFController(p, i, d, f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        // Motors DriveTrain
        DcMotor leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotor leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotor rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotor rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        //Banda
        DcMotor banda = hardwareMap.get(DcMotorEx.class, "banda");

        // Motors Lift
        DcMotor rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        DcMotor leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");

        // Motors Intake
        DcMotor intake = hardwareMap.get(DcMotorEx.class, "intake");

        // Servos Intake
        Servo rightIntakeSv = hardwareMap.get(Servo.class, "rightIntakeSv");
        Servo leftIntakeSv = hardwareMap.get(Servo.class, "leftIntakeSv");

        //Agatat
        CRServo PullupServo = hardwareMap.get(CRServo.class, "PullupServo");
        AnalogInput PullUpEncoder = hardwareMap.get(AnalogInput.class, "PullUpEncoder");

        // Servos Lift
//        Servo leftLiftSv = hardwareMap.get(Servo.class, "leftLiftSv");
//        Servo rightLiftSv = hardwareMap.get(Servo.class, "rightLiftSv");
//
//        DistanceSensor sensorDistance = hardwareMap.get(DistanceSensor.class, "sensorDistance");

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

        RevColorSensorV3 colorSensor = (RevColorSensorV3) hardwareMap.colorSensor.get("colorSensor");
        RevColorSensorV3 colorSensor2 = (RevColorSensorV3) hardwareMap.colorSensor.get("colorSensor");

        PullupServo.setPower(-0.05);


//        leftLiftSv.setPosition(0.70);
//        rightLiftSv.setPosition(0.70);


        leftIntakeSv.setPosition(0);
        rightIntakeSv.setPosition(0);

        Pose2d beginPose = new Pose2d(-38, 61, -Math.PI / 2);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);


        waitForStart();
        ElapsedTime timerTotal = new ElapsedTime();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            controller.setPIDF(p, i, d, f);
            double liftPos = (rightLift.getCurrentPosition() + leftLift.getCurrentPosition()) / 2;
            double pidf = controller.calculate(liftPos, target);
            double motorRelativeError = Math.abs(leftLift.getCurrentPosition() - rightLift.getCurrentPosition()) > 10 ? leftLift.getCurrentPosition() - rightLift.getCurrentPosition() : 0;
            double rightLiftPower = pidf + relatieP * motorRelativeError;
            double leftLiftPower = pidf - relatieP * motorRelativeError;
            double fixer = Math.max(rightLiftPower, Math.max(leftLiftPower, 1));

            rightLift.setPower(rightLiftPower / fixer / 1.5);
            leftLift.setPower(leftLiftPower / fixer / 1.5);


            switch (robotState) {
                case START:
//                    sleep(10000);
                    Actions.runBlocking(
                            drive.actionBuilder(beginPose)
                                    .strafeTo(new Vector2d(-49,32))
                                    .strafeTo(new Vector2d(-49,36))
                                    .strafeToLinearHeading(new Vector2d(48,36),0)
                                    .strafeTo(new Vector2d(53,31))

//                                    .lineToX(58)
                                    .build());

                    timer.reset();
                    robotState = RobotState.COLLECTING;

                    break;
                case COLLECTING:
                    target = 2400;
                    if(timer.seconds()>2.5) {
                        target = -70;
                        if (bolcase == 2) robotState = RobotState.IDLE;
                        else robotState = RobotState.RETRACTING;

                    }

                    break;
                case RETRACTING:
                    timer.reset();
                    Actions.runBlocking(new SequentialAction(
                            new ParallelAction(
                                    (telemetryPacket) -> {
                                        leftIntakeSv.setPosition(IntakeMidSvPos);
                                        rightIntakeSv.setPosition(IntakeMidSvPos);
                                        banda.setPower(1);
                                        return false;

                                    }
                            ),
                            drive.actionBuilder(new Pose2d(53, 31, 0))
                                    .strafeTo(new Vector2d(10,11))
                                    .strafeTo(new Vector2d(-48,11))
                                    .build())

                    );

//                    banda.setPower(0);
//                    if(timer.seconds()>5)
                    timer.reset();
                        robotState = RobotState.NEUTRAL;
                    break;
                case NEUTRAL:

                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(-48, 11, 0))
                                    .strafeTo(new Vector2d(10,11))
                                    .strafeTo(new Vector2d(53,31))
//                                    .lineToX(58)
                                    .build());
                    bolcase = 2;
                    if(timer.seconds()>6){
                        timer.reset();
                        robotState = RobotState.COLLECTING;
                    }
                    break;
                case IDLE:
                    idle();
                    break;
            }

//            if (gamepad2.dpad_down && robotState != RobotState.START) {
//                robotState = RobotState.START;
//            }


//            if (gamepad1.right_bumper == TRUE) {
//                leftFront.setPower(frontLeftPower/3);
//                leftRear.setPower(backLeftPower/3);
//                rightFront.setPower(frontRightPower/3);
//                rightRear.setPower(backRightPower/3);
//            } else {
//                leftFront.setPower(frontLeftPower);
//                leftRear.setPower(backLeftPower);
//                rightFront.setPower(frontRightPower);
//                rightRear.setPower(backRightPower);
//            }

//            if ((colorSensor.getDistance(DistanceUnit.CM) + colorSensor2.getDistance(DistanceUnit.CM)) / 2 < 2.65) {
//                pixel += 1;
//                while ((colorSensor.getDistance(DistanceUnit.CM) + colorSensor2.getDistance(DistanceUnit.CM)) / 2 < 2.99) {
//                }
//
//            }
//
//            double ServVit = gamepad2.left_stick_y;
//            PullupServo.setPower(ServVit);
//
//
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


            telemetry.addData("Distance: ", colorSensor.getDistance(DistanceUnit.CM) + " cm");
            telemetry.addData("Dist", Dist.getDistance(DistanceUnit.CM));

            telemetry.addData("Distance 2: ", colorSensor2.getDistance(DistanceUnit.CM) + " cm");
            telemetry.addData("Average", (colorSensor.getDistance(DistanceUnit.CM) + colorSensor2.getDistance(DistanceUnit.CM)) / 2);
            telemetry.addData("Nr pixeli", pixel);
            telemetry.addData("Stadiu Robot", robotState);
            telemetry.addData("target ", target);
            telemetry.addData("pos ", liftPos);
//            telemetry.addData("leftPos", leftLift.getCurrentPosition());
//            telemetry.addData("rightPos", rightLift.getCurrentPosition());
//            telemetry.addData("relativeerror", motorRelativeError);
//            telemetry.addData("Distance: ", sensorDistance.getDistance(DistanceUnit.CM) + " cm");
            telemetry.addData("RightFront", rightFront.getCurrentPosition());
            telemetry.addData("RightRear", rightRear.getCurrentPosition());
            telemetry.addData("LeftFront", leftFront.getCurrentPosition());
            telemetry.addData("LeftRear", leftRear.getCurrentPosition());
            telemetry.addData("Timp ", timer.milliseconds());

            telemetry.update();

        }
    }
}
