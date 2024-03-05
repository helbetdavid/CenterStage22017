//
//package org.firstinspires.ftc.teamcode.Ono;
//
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//@TeleOp(name = "fakemariusmoga3 (Blocks to Java)")
//public class fakemariusmoga3 extends LinearOpMode {
//
//    private DcMotor leftRear;
//    private DcMotor leftLift;
//    private DcMotor rightFront;
//    private DcMotor leftFront;
//    private DcMotor rightRear;
//    private DcMotor banda;
//    private Servo rightIntakeSv;
//    private IMU imu_IMU;
//    private Servo leftIntakeSv;
//    private DcMotor rightLift;
//    private DcMotor intake;
//    private DistanceSensor DistSpateDr;
//    private DistanceSensor DistSpateSt;
//    private Servo cutie;
//    private DistanceSensor DistFataDr;
//
//    int gp1_stanga_orizontal;
//    int z;
//    int in_plus_motoare_dreapta;
//    int in_plus_motoare_stanga;
//    double y;
//    double nrz;
//    double x;
//    double gp1_dreapta_orizontal;
//    float unghinou222;
//    int suma;
//    double t;
//    double distanta_stanga2;
//    double distanta_dreapta2;
//    double w;
//    long msinit;
//    int rotatii222;
//    int nrx;
//    double gp1_stanga_vertical;
//    double unghivechi111;
//    double putere555;
//    double nry;
//    double p;
//    int i;
//    int d;
//    double gp1_dreapta_vertical;
//    double kp;
//    double ki;
//    double kd;
//    int sac;
//    int lasterror;
//    int contor;
//    int mediesac;
//    double error;
//    double _C8_9B;
//
//    /**
//     * Describe this function...
//     */
//    private void wait(int timems) {
//        long timems1;
//
//        // Get the current time in milliseconds. The value returned represents
//        // the number of milliseconds since midnight, January 1, 1970 UTC.
//        timems1 = System.currentTimeMillis();
//        // Get the current time in milliseconds. The value returned represents
//        // the number of milliseconds since midnight, January 1, 1970 UTC.
//        while (System.currentTimeMillis() - timems1 < timems && opModeIsActive()) {
//        }
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void initmotoare() {
//        in_plus_motoare_dreapta = 0;
//        leftRear.setDirection(DcMotor.Direction.REVERSE);
//        leftLift.setDirection(DcMotor.Direction.REVERSE);
//    }
//
//    /**
//     * Describe this function...
//     */
//    // Get the current time in milliseconds. The value returned represents
//    // the number of milliseconds since midnight, January 1, 1970 UTC.
//    private double globalizarems() {
//        return System.currentTimeMillis() - msinit;
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void read_1_gamepad_1() {
//        double siny;
//        double cosy;
//
//        x = 0.35;
//        if (gamepad1.dpad_right) {
//            gp1_stanga_orizontal = (int) x;
//        } else if (gamepad1.dpad_left) {
//            gp1_stanga_orizontal = (int) -x;
//        } else {
//            gp1_stanga_orizontal = 0;
//        }
//        if (gamepad1.dpad_up) {
//            gp1_stanga_vertical = x;
//        } else if (gamepad1.dpad_down) {
//            gp1_stanga_vertical = -x;
//        } else {
//            gp1_stanga_vertical = 0;
//        }
//        if (gamepad1.circle) {
//            gp1_dreapta_orizontal = x;
//        } else if (gamepad1.square) {
//            gp1_dreapta_orizontal = -x;
//        } else {
//            gp1_dreapta_orizontal = 0;
//        }
//        if (gamepad1.triangle) {
//            gp1_dreapta_vertical = x;
//        } else if (gamepad1.cross) {
//            gp1_dreapta_vertical = -x;
//        } else {
//            gp1_dreapta_vertical = 0;
//        }
//        y = unghie();
//        siny = Math.sin(y / 180 * Math.PI) * Math.sin(y / 180 * Math.PI);
//        cosy = Math.cos(y / 180 * Math.PI) * Math.cos(y / 180 * Math.PI);
//        rightFront.setPower(gp1_stanga_vertical * cosy + -gp1_stanga_vertical * siny + -gp1_stanga_orizontal * cosy + -gp1_stanga_orizontal * siny + -gp1_dreapta_orizontal);
//        leftFront.setPower(gp1_stanga_vertical * cosy + gp1_stanga_vertical * siny + gp1_stanga_orizontal * cosy + -gp1_stanga_orizontal * siny + gp1_dreapta_orizontal);
//        leftRear.setPower(gp1_stanga_vertical * cosy + -gp1_stanga_vertical * siny + -gp1_stanga_orizontal * cosy + -gp1_stanga_orizontal * siny + gp1_dreapta_orizontal);
//        rightRear.setPower(gp1_stanga_vertical * cosy + gp1_stanga_vertical * siny + gp1_stanga_orizontal * cosy + -gp1_stanga_orizontal * Math.sin(y / 180 * Math.PI) * siny + -gp1_dreapta_orizontal);
//
////        nrx = gamepad1.right_stick_button;
//
//        telemetry.addData("unghi curent", unghie());
//        telemetry.addData("nrz", nrz);
//        telemetry.addData("nry", nry);
//        telemetry.addData("ultimul unghi", unghinou222);
//        telemetry.addData("ungi de eroare", nry - nrz);
//        telemetry.update();
//        if (gamepad1.triangle) {
//            banda.setPower(1);
//        } else if (gamepad1.cross) {
//            banda.setPower(-1);
//        } else {
//            banda.setPower(0);
//        }
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void stopmotor() {
//        rightFront.setPower(0);
//        leftFront.setPower(0);
//        leftRear.setPower(0);
//        rightRear.setPower(0);
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void initunghie() {
//        // Initialize the IMU with non-default settings. To use this block,
//        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
//        // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
//        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
//        // the REV Robotics logo is facing and the direction that the USB ports are facing.
//        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
//        rotatii222 = 0;
//        while (unghie() != 0) {
//            unghivechi111 = 0;
//            unghivechi111 = unghie();
//            wait(50);
//        }
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void megeoci1() {
//        telemetry.addData("inainte acelerare", globalizarems());
//        acelerare(1000);
//        telemetry.addData("dupa acelerare", globalizarems());
//        wait(800);
//        telemetry.addData("inainte incetinire", globalizarems());
//        deacelerare(1000);
//        telemetry.addData("dupa incetinire", globalizarems());
//        telemetry.update();
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void mers_cu_distanta(double distanta) {
//        x = media_aritmetica_2();
//        rightRear.setPower(0.5);
//        rightFront.setPower(0.5);
//        leftFront.setPower(0.5);
//        leftRear.setPower(0.5);
//        while (!(media_aritmetica_2() - x >= distanta * 74.182)) {
//        }
//        stopmotor();
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void deacelerare(int timems) {
//        x = 100;
//        nrx = 200;
//        for (int count = 0; count < 100; count++) {
//            x += -1;
//            wait(timems / 100);
//            rightFront.setPower(x / nrx);
//            leftFront.setPower(x / nrx);
//            leftRear.setPower(x / nrx);
//            rightRear.setPower(x / nrx);
//        }
//    }
//
//    /**
//     * This function is executed when this OpMode is selected from the Driver Station.
//     */
//    @Override
//    public void runOpMode() {
//        double xinitial;
//
//        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
//        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
//        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
//        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
//        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
//        banda = hardwareMap.get(DcMotor.class, "banda");
//        rightIntakeSv = hardwareMap.get(Servo.class, "rightIntakeSv");
//        imu_IMU = hardwareMap.get(IMU.class, "imu");
//        leftIntakeSv = hardwareMap.get(Servo.class, "leftIntakeSv");
//        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
//        intake = hardwareMap.get(DcMotor.class, "intake");
//        DistSpateDr = hardwareMap.get(DistanceSensor.class, "DistSpateDr");
//        DistSpateSt = hardwareMap.get(DistanceSensor.class, "DistSpateSt");
//        cutie = hardwareMap.get(Servo.class, "cutie");
//        DistFataDr = hardwareMap.get(DistanceSensor.class, "DistFataDr");
//
//        initmotoare();
//        wait(2000);
//        waitForStart();
//        if (opModeIsActive()) {
//            xinitial = unghie();
//            // Get the current time in milliseconds. The value returned represents
//            // the number of milliseconds since midnight, January 1, 1970 UTC.
//            msinit = System.currentTimeMillis();
//            putere555 = 0.5;
//            initunghie();
//            coboara_intake(27);
//        }
//        while (opModeIsActive()) {
//            read_2_gamepad_1();
//            wait(25);
//        }
//    }
//
//    /**
//     * Describe this function...
//     */
//    private double media_aritmetica_2() {
//        int qqq;
//
//        qqq = rightRear.getCurrentPosition() + rightRear.getCurrentPosition();
//        return qqq / -2;
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void acelerare(int timems) {
//        x = 0;
//        nrx = 200;
//        for (int count2 = 0; count2 < 100; count2++) {
//            x += 1;
//            wait(timems / 100);
//            rightFront.setPower(x / nrx);
//            leftFront.setPower(x / nrx);
//            leftRear.setPower(x / nrx);
//            rightRear.setPower(x / nrx);
//        }
//    }
//
//    /**
//     * Describe this function...
//     */
//    private double unghie() {
//        menajerieunghi();
//        return (360 * rotatii222 + unghinou222) - unghivechi111;
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void menajerieunghi() {
//        double unghivechi222;
//        unghivechi222 = imu_IMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
//
//        unghinou222 = imu_IMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
//        if (unghivechi222 > 170 && unghinou222 < -170) {
//            rotatii222 += 1;
//        }
//        if (unghivechi222 < -170 && unghinou222 > 170) {
//            rotatii222 += -1;
//        }
//        unghivechi222 = unghinou222;
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void mergi_pe_giro(double distanta, double puterea) {
//        x = media_aritmetica_2();
//        y = unghie();
//        z = 0;
//        leftRear.setPower(puterea);
//        rightRear.setPower(puterea);
//        leftFront.setPower(puterea);
//        rightFront.setPower(puterea);
//        while (!(media_aritmetica_2() - x >= distanta * 74.182)) {
//            z += ((unghie() - y) / 1000) * 2;
//            rightFront.setPower(puterea - z);
//            leftFront.setPower(puterea + z);
//            leftRear.setPower(puterea + z);
//            rightRear.setPower(puterea - z);
//            wait(10);
//        }
//        stopmotor();
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void print_all_gamepad_1() {
//        telemetry.addData("1x", gamepad1.touchpad_finger_1_x);
//        telemetry.addData("1y", gamepad1.touchpad_finger_1_y);
//        telemetry.addData("2x", gamepad1.touchpad_finger_2_x);
//        telemetry.addData("2y", gamepad1.touchpad_finger_2_y);
//        telemetry.addData("Guide", gamepad1.guide);
//        telemetry.addData("Options", gamepad1.options);
//        telemetry.addData("Share", gamepad1.share);
//        telemetry.addData("Start", gamepad1.start);
//        telemetry.addData("Touchpad", gamepad1.touchpad);
//        telemetry.addData("TouchpadFinger1", gamepad1.touchpad_finger_1);
//        telemetry.addData("TouchpadFinger2", gamepad1.touchpad_finger_2);
//        telemetry.addData("X", gamepad1.x);
//        telemetry.addData("Y", gamepad1.y);
//        telemetry.update();
//    }
//
//    /**
//     * Describe this function...
//     */
//    private float rawangle() {
//        return imu_IMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void gyro_viraj(int unghiviraj, double puterea) {
//        if (unghiviraj >= unghie()) {
//            while (unghie() < unghiviraj) {
//                rightFront.setPower(puterea);
//                leftFront.setPower(-puterea);
//                leftRear.setPower(-puterea);
//                rightRear.setPower(puterea);
//            }
//        } else {
//            while (unghie() > unghiviraj) {
//                leftFront.setPower(puterea);
//                rightFront.setPower(-puterea);
//                rightRear.setPower(-puterea);
//                leftRear.setPower(puterea);
//            }
//        }
//        stopmotor();
//        wait(300);
//        if (unghiviraj >= unghie()) {
//            while (unghie() < unghiviraj) {
//                rightFront.setPower(puterea - 0.3);
//                leftFront.setPower(-(puterea - 0.3));
//                leftRear.setPower(-(puterea - 0.3));
//                rightRear.setPower(puterea - 0.3);
//            }
//        } else {
//            while (unghie() > unghiviraj) {
//                leftFront.setPower(puterea - 0.3);
//                rightFront.setPower(-(puterea - 0.3));
//                rightRear.setPower(-(puterea - 0.3));
//                leftRear.setPower(puterea - 0.3);
//            }
//        }
//        stopmotor();
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void coboara_intake(float procent) {
//        double intake_stanga_minim;
//        double intake_dreapta_minim;
//        double intake_ra;
//
//        intake_stanga_minim = 0.51;
//        intake_dreapta_minim = 0.51;
//        intake_ra = -0.4 * (procent / 100);
//        leftIntakeSv.setPosition(intake_stanga_minim + intake_ra);
//        rightIntakeSv.setPosition(intake_dreapta_minim + intake_ra);
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void glisiere() {
//        leftLift.setPower(0.5);
//        rightLift.setPower(0.5);
//        wait(2500);
//        leftLift.setPower(0);
//        rightLift.setPower(0);
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void mergi_pe_giro_pib(int distanta, double puterea) {
//        x = media_aritmetica_2();
//        y = unghie();
//        z = 0;
//        i = 0;
//        d = 0;
//        kp = 0.03;
//        ki = 0.0012;
//        kd = 0.07;
//        sac = 0;
//        lasterror = 0;
//        contor = 0;
//        mediesac = 0;
//        leftRear.setPower(puterea);
//        rightRear.setPower(puterea);
//        leftFront.setPower(puterea);
//        rightFront.setPower(puterea);
//        while (!(media_aritmetica_2() - x >= distanta * 74.182)) {
//            error = unghie() - y;
//            p = error * kp;
//            sac += error;
//            i = (int) (sac * ki);
//            d = (int) ((error - lasterror) * kd);
//            _C8_9B = p + i + d;
//            lasterror = (int) error;
//            rightFront.setPower(puterea - _C8_9B);
//            leftFront.setPower(puterea + _C8_9B);
//            leftRear.setPower(puterea + _C8_9B);
//            rightRear.setPower(puterea - _C8_9B);
//            contor += 1;
//            wait(10);
//        }
//        stopmotor();
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void read_2_gamepad_1() {
//        // TODO: Enter the type for variable named TouchpadFinger2X
//        UNKNOWN_TYPE TouchpadFinger2X;
//
//        x = 0.45;
//        TouchpadFinger2X = 0;
//        if (gamepad1.touchpad_finger_1) {
//            gp1_stanga_orizontal = (int) (gamepad1.touchpad_finger_1_x * x);
//        } else {
//            if (gamepad1.dpad_right) {
//                gp1_stanga_orizontal = (int) x;
//            } else if (gamepad1.dpad_left) {
//                gp1_stanga_orizontal = (int) -x;
//            } else {
//                gp1_stanga_orizontal = 0;
//            }
//        }
//        if (gamepad1.touchpad_finger_1) {
//            gp1_stanga_vertical = gamepad1.touchpad_finger_1_y * x;
//        } else {
//            if (gamepad1.dpad_up) {
//                gp1_stanga_vertical = x;
//            } else if (gamepad1.dpad_down) {
//                gp1_stanga_vertical = -x;
//            } else {
//                gp1_stanga_vertical = 0;
//            }
//        }
//        if (gamepad1.circle) {
//            gp1_dreapta_orizontal = x;
//        } else if (gamepad1.square) {
//            gp1_dreapta_orizontal = -x;
//        } else {
//            gp1_dreapta_orizontal = 0;
//        }
//        if (gamepad1.triangle) {
//            gp1_dreapta_vertical = x;
//        } else if (gamepad1.cross) {
//            gp1_dreapta_vertical = -x;
//        } else {
//            gp1_dreapta_vertical = 0;
//        }
//        rightFront.setPower(gp1_stanga_vertical + -gp1_stanga_orizontal + -gp1_dreapta_orizontal + 0);
//        leftFront.setPower(gp1_stanga_vertical + gp1_stanga_orizontal + gp1_dreapta_orizontal + 0);
//        leftRear.setPower(gp1_stanga_vertical + -gp1_stanga_orizontal + gp1_dreapta_orizontal + 0);
//        rightRear.setPower(gp1_stanga_vertical + gp1_stanga_orizontal + -gp1_dreapta_orizontal + 0);
//        nrx = gamepad1.right_stick_button;
//        telemetry.addData("unghi curent", unghie());
//        telemetry.addData("enc st gls", leftLift.getCurrentPosition());
//        telemetry.addData("enc dr gls", rightLift.getCurrentPosition());
//        telemetry.addData("encoder lateral", leftRear.getCurrentPosition());
//        telemetry.addData("distanta senzor1", redmmst());
//        telemetry.addData("distanta senzor2", redmmdr());
//        telemetry.addData("corectare dreapta", in_plus_motoare_dreapta);
//        telemetry.addData("corectare stanga", in_plus_motoare_stanga);
//        telemetry.addData("ds dr", distanta_dreapta2);
//        telemetry.addData("ds st", distanta_stanga2);
//        telemetry.update();
//        if (gamepad1.left_trigger > 0.9) {
//        }
//        if (gamepad1.right_trigger > 0.9) {
//            mentine_distanta3(130, 0.21);
//        }
//        if (gamepad1.ps) {
//            initunghie();
//        }
//        if (gamepad1.options) {
//            initunghie();
//            wait(2000);
//            coboara_intake(27);
//            mentine_distanta3(130, 0.21);
//            intake.setPower(1);
//            banda.setPower(1);
//            wait(500);
//            mergi_pe_giro_pib_fs_ono(100, 0.5);
//            coboara_intake(0);
//            wait(250);
//            mergi_pe_giro_pib_fs_ono(-50, 0.5);
//            mergi_pe_giro_pib_fs_ono(1950, 0.9);
//            intake.setPower(0);
//            mentine_distanta2(100, 0.5);
//            mentine_distanta2(100, 0.2);
//            coboara_intake(50);
//            banda.setPower(0);
//            side_step_horia_pid(-420, 0.6);
//            mergi_pe_giro_pib_fs_ono(250, 0.3);
//            wait(500);
//            slide_iustin(1, 350, 0.4);
//            side_step_horia_pid(300, 0.55);
//            gyro_viraj_ono(0, 0.5);
//            mergi_pe_giro_pib_fs_ono(-1300, 0.9);
//            mentine_distanta2(100, 0.5);
//            mentine_distanta2(100, 0.2);
//            mergi_pe_giro_pib_fs_ono(-400, 0.3);
//            wait(250);
//            intake.setPower(1);
//            banda.setPower(1);
//            mentine_distanta3(150, 0.21);
//            coboara_intake(20);
//            mergi_pe_giro_pib_fs_ono(80, 0.5);
//            mergi_pe_giro_pib_fs_ono(-100, 0.5);
//            wait(100);
//            mergi_pe_giro_pib_fs_ono(100, 0.5);
//            mergi_pe_giro_pib_fs_ono(-100, 0.5);
//            wait(100);
//            mergi_pe_giro_pib_fs_ono(100, 0.5);
//            coboara_intake(0);
//            wait(150);
//            mergi_pe_giro_pib_fs_ono(-100, 0.5);
//            mergi_pe_giro_pib_fs_ono(1980, 0.9);
//            intake.setPower(0);
//            mentine_distanta2(100, 0.5);
//            mentine_distanta2(100, 0.2);
//            coboara_intake(50);
//            banda.setPower(0);
//            side_step_horia_pid(-400, 0.6);
//            mergi_pe_giro_pib_fs_ono(200, 0.3);
//            stopmotor();
//        }
//        if (gamepad1.cross) {
//            glisiere();
//            cutie_pixel();
//        } else {
//        }
//        if (gamepad1.triangle) {
//            leftLift.setPower(-0.5);
//            rightLift.setPower(-0.5);
//            wait(100);
//            leftLift.setPower(0);
//            rightLift.setPower(0);
//        } else {
//        }
//        if (gamepad1.right_bumper) {
//            gyro_viraj(-90, 0.5);
//        }
//        if (gamepad1.left_bumper) {
//            side_step_horia_pid(700, 0.5);
//        }
//        if (gamepad1.right_stick_button) {
//            mentine_distanta3(200, 0.3);
//        }
//        if (gamepad1.left_stick_button) {
//            telemetry.addData("medie arit senz", (DistSpateDr.getDistance(DistanceUnit.MM) + DistSpateSt.getDistance(DistanceUnit.MM)) / 2);
//            telemetry.update();
//            senzoreala_giroreasca_horiasca(0.3, 400);
//        }
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void cutie_pixel() {
//        cutie.setPosition(0.5);
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void mergi_pe_giro_pib_fs_ono(int distanta, double puterea) {
//        if (distanta < 0) {
//            w = -1;
//        } else {
//            w = 1;
//        }
//        x = media_aritmetica_2();
//        y = unghie();
//        z = 0;
//        i = 0;
//        d = 0;
//        kp = 0.03;
//        ki = 0.0012;
//        kd = 0.07;
//        sac = 0;
//        lasterror = 0;
//        contor = 0;
//        mediesac = 0;
//        leftRear.setPower(puterea * w);
//        rightRear.setPower(puterea * w);
//        leftFront.setPower(puterea * w);
//        rightFront.setPower(puterea * w);
//        while (!((media_aritmetica_2() - x) * w >= distanta * 74.182 * w)) {
//            error = unghie() - y;
//            p = error * kp;
//            sac += error;
//            i = (int) (sac * ki);
//            d = (int) ((error - lasterror) * kd);
//            _C8_9B = p + i + d;
//            lasterror = (int) error;
//            rightFront.setPower(puterea * w - _C8_9B);
//            leftFront.setPower(puterea * w + _C8_9B);
//            leftRear.setPower(puterea * w + _C8_9B);
//            rightRear.setPower(puterea * w - _C8_9B);
//            contor += 1;
//            wait(10);
//        }
//        stopmotor();
//    }
//
//    /**
//     * Describe this function...
//     */
//    private double redmmst() {
//        return DistSpateSt.getDistance(DistanceUnit.MM);
//    }
//
//    /**
//     * Describe this function...
//     */
//    private int distanta_stanga(int precizie) {
//        suma = 0;
//        for (int count3 = 0; count3 < precizie; count3++) {
//            suma += redmmst();
//        }
//        return suma / precizie;
//    }
//
//    /**
//     * Describe this function...
//     */
//    private int distanta_iustin(int precizie) {
//        suma = 0;
//        for (int count4 = 0; count4 < precizie; count4++) {
//            suma += marime_pantof_iustin();
//        }
//        return suma / precizie;
//    }
//
//    /**
//     * Describe this function...
//     */
//    private double redmmdr() {
//        return DistSpateDr.getDistance(DistanceUnit.MM);
//    }
//
//    /**
//     * Describe this function...
//     */
//    private int distanta_dreapta(int precizie) {
//        suma = 0;
//        for (int count5 = 0; count5 < precizie; count5++) {
//            suma += redmmdr();
//        }
//        return suma / precizie;
//    }
//
//    /**
//     * Describe this function...
//     */
//    private double marime_pantof_iustin() {
//        return DistFataDr.getDistance(DistanceUnit.MM);
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void gyro_viraj_ono(int unghiviraj, double puterea) {
//        t = globalizarems();
//        w = 0.18;
//        z = 0;
//        while (Math.abs(unghie() - unghiviraj) > 0.75 || z < 10) {
//            while (Math.abs(unghie() - unghiviraj) > 0.75) {
//                z = 0;
//                if (unghiviraj >= unghie()) {
//                    y = 1;
//                } else {
//                    y = -1;
//                }
//                x = (puterea * 0.72 + 0.1) * ((unghiviraj - unghie()) / 150) + w * y;
//                if (globalizarems() - t > 5000) {
//                    z = 1000;
//                }
//                rightFront.setPower(x);
//                leftFront.setPower(-x);
//                leftRear.setPower(-x);
//                rightRear.setPower(x);
//            }
//            stopmotor();
//            z += 1;
//            w += -0.005;
//        }
//        stopmotor();
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void mentine_distanta(double distanta) {
//        double eroare_distanta;
//
//        distanta_stanga2 = distanta_stanga(1);
//        distanta_dreapta2 = distanta_dreapta(1);
//        if (distanta_dreapta2 + distanta_stanga2 > distanta * 4) {
//            in_plus_motoare_dreapta = 0;
//            in_plus_motoare_stanga = 0;
//        } else {
//            eroare_distanta = distanta_dreapta2 - distanta_stanga2;
//            in_plus_motoare_dreapta = (int) ((distanta / distanta_dreapta2 - 1) / 1);
//            in_plus_motoare_stanga = (int) ((distanta / distanta_stanga2 - 1) / 1);
//        }
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void mergi_spate_gyro_pib(int distanta, double puterea) {
//        x = media_aritmetica_2();
//        y = unghie();
//        z = 0;
//        i = 0;
//        d = 0;
//        kp = 0.03;
//        ki = 0.0012;
//        kd = 0.07;
//        sac = 0;
//        lasterror = 0;
//        contor = 0;
//        mediesac = 0;
//        leftRear.setPower(0 - puterea);
//        rightRear.setPower(0 - puterea);
//        leftFront.setPower(0 - puterea);
//        rightFront.setPower(0 - puterea);
//        while (!(-(media_aritmetica_2() - x) >= distanta * 74.182)) {
//            error = unghie() - y;
//            p = error * kp;
//            sac += error;
//            i = (int) (sac * ki);
//            d = (int) ((error - lasterror) * kd);
//            _C8_9B = p + i + d;
//            lasterror = (int) error;
//            rightFront.setPower(-(puterea + _C8_9B));
//            leftFront.setPower(-(puterea - _C8_9B));
//            leftRear.setPower(-(puterea - _C8_9B));
//            rightRear.setPower(-(puterea + _C8_9B));
//            contor += 1;
//            wait(10);
//        }
//        stopmotor();
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void mentine_distanta3(int distanta, double puterea) {
//        boolean ok;
//
//        ok = true;
//        t = globalizarems();
//        p = puterea;
//        x = 0;
//        distanta_stanga2 = distanta_stanga(1);
//        distanta_dreapta2 = distanta_dreapta(1);
//        if (distanta_dreapta2 + distanta_stanga2 < 1111) {
//            while (!(Math.abs(distanta_dreapta2 - distanta_stanga2) < 3 && Math.abs((distanta_dreapta2 + distanta_stanga2) - distanta * 2) < 3 || x >= 10 || !ok || !opModeIsActive())) {
//                x += 1;
//                while (!(Math.abs(distanta_dreapta2 - distanta_stanga2) < 3 && Math.abs((distanta_dreapta2 + distanta_stanga2) - distanta * 2) < 3 || !ok || !opModeIsActive())) {
//                    x = 0;
//                    if (Math.abs(distanta_dreapta2 - distanta) > 2) {
//                        if (distanta_dreapta2 < distanta) {
//                            rightFront.setPower(puterea);
//                            rightRear.setPower(puterea);
//                        } else {
//                            rightFront.setPower(-puterea);
//                            rightRear.setPower(-puterea);
//                        }
//                    } else {
//                        rightFront.setPower(0);
//                        rightRear.setPower(0);
//                    }
//                    if (Math.abs(distanta_stanga2 - distanta) > 2) {
//                        if (distanta_stanga2 < distanta) {
//                            leftFront.setPower(puterea);
//                            leftRear.setPower(puterea);
//                        } else {
//                            leftFront.setPower(-puterea);
//                            leftRear.setPower(-puterea);
//                        }
//                    } else {
//                        leftFront.setPower(0);
//                        leftRear.setPower(0);
//                    }
//                    distanta_stanga2 = distanta_stanga(1);
//                    distanta_dreapta2 = distanta_dreapta(1);
//                    telemetry.addData("putere curenta", puterea);
//                    telemetry.update();
//                    if (globalizarems() - t >= 3000) {
//                        ok = false;
//                    }
//                    puterea = p - (globalizarems() - t) / 15000;
//                }
//                distanta_stanga2 = distanta_stanga(1);
//                distanta_dreapta2 = distanta_dreapta(1);
//                stopmotor();
//            }
//        }
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void mentine_distanta2(int timpms, double puterea) {
//        double distanta;
//        double v;
//
//        x = media_aritmetica_2();
//        v = globalizarems();
//        y = unghie();
//        z = 0;
//        i = 0;
//        d = 0;
//        kp = 0.03;
//        ki = 0.0012;
//        kd = 0.07;
//        sac = 0;
//        lasterror = 0;
//        contor = 0;
//        mediesac = 0;
//        while (globalizarems() - v < timpms) {
//            distanta = (media_aritmetica_2() - x) * -1;
//            if (distanta < 0) {
//                w = -1;
//            } else {
//                w = 1;
//            }
//            leftRear.setPower(puterea * w);
//            rightRear.setPower(puterea * w);
//            leftFront.setPower(puterea * w);
//            rightFront.setPower(puterea * w);
//            while (!((media_aritmetica_2() - x) * w >= distanta * w)) {
//                error = unghie() - y;
//                p = error * kp;
//                sac += error;
//                i = (int) (sac * ki);
//                d = (int) ((error - lasterror) * kd);
//                _C8_9B = p + i + d;
//                lasterror = (int) error;
//                rightFront.setPower(puterea * w - _C8_9B);
//                leftFront.setPower(puterea * w + _C8_9B);
//                leftRear.setPower(puterea * w + _C8_9B);
//                rightRear.setPower(puterea * w - _C8_9B);
//                contor += 1;
//                wait(10);
//            }
//            stopmotor();
//        }
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void side_step_horia_pid(int distanta, double puterea) {
//        if (distanta < 0) {
//            w = -1;
//        } else {
//            w = 1;
//        }
//        x = -leftRear.getCurrentPosition();
//        y = unghie();
//        z = 0;
//        i = 0;
//        d = 0;
//        kp = 0.03;
//        ki = 0.0012;
//        kd = 0.07;
//        sac = 0;
//        lasterror = 0;
//        contor = 0;
//        mediesac = 0;
//        leftRear.setPower(-(puterea * w));
//        rightRear.setPower(puterea * w);
//        leftFront.setPower(puterea * w);
//        rightFront.setPower(-(puterea * w));
//        while (!(Math.abs(-leftRear.getCurrentPosition() - x) >= Math.abs(distanta * 74.182))) {
//            error = unghie() - y;
//            p = error * kp;
//            sac += error;
//            i = (int) (sac * ki);
//            d = (int) ((error - lasterror) * kd);
//            _C8_9B = p + i + d;
//            lasterror = (int) error;
//            rightFront.setPower(-(puterea * w + _C8_9B));
//            leftFront.setPower(puterea * w + _C8_9B);
//            leftRear.setPower(-(puterea * w + _C8_9B));
//            rightRear.setPower(puterea * w - _C8_9B);
//            contor += 1;
//            wait(10);
//        }
//        stopmotor();
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void slide_iustin(int sens, int distanta, double puterea) {
//        if (sens < 0) {
//            w = -1;
//        } else {
//            w = 1;
//        }
//        x = -leftRear.getCurrentPosition();
//        y = unghie();
//        z = 0;
//        i = 0;
//        d = 0;
//        kp = 0.03;
//        ki = 0.0012;
//        kd = 0.07;
//        sac = 0;
//        lasterror = 0;
//        contor = 0;
//        mediesac = 0;
//        leftRear.setPower(-(puterea * w));
//        rightRear.setPower(puterea * w);
//        leftFront.setPower(puterea * w);
//        rightFront.setPower(-(puterea * w));
//        while (!(distanta_iustin(1) >= distanta)) {
//            error = unghie() - y;
//            p = error * kp;
//            sac += error;
//            i = (int) (sac * ki);
//            d = (int) ((error - lasterror) * kd);
//            _C8_9B = p + i + d;
//            lasterror = (int) error;
//            rightFront.setPower(-(puterea * w + _C8_9B));
//            leftFront.setPower(puterea * w + _C8_9B);
//            leftRear.setPower(-(puterea * w + _C8_9B));
//            rightRear.setPower(puterea * w - _C8_9B);
//            contor += 1;
//            wait(10);
//        }
//        stopmotor();
//    }
//
//    /**
//     * Describe this function...
//     */
//    private void senzoreala_giroreasca_horiasca(double puterea, int dist_senzori) {
//        w = -1;
//        x = media_aritmetica_2();
//        y = unghie();
//        z = 0;
//        i = 0;
//        d = 0;
//        kp = 0.03;
//        ki = 0.0012;
//        kd = 0.07;
//        sac = 0;
//        lasterror = 0;
//        contor = 0;
//        mediesac = 0;
//        leftRear.setPower(puterea * w);
//        rightRear.setPower(puterea * w);
//        leftFront.setPower(puterea * w);
//        rightFront.setPower(puterea * w);
//        while (!((DistSpateDr.getDistance(DistanceUnit.MM) + DistSpateSt.getDistance(DistanceUnit.MM)) / 2 <= dist_senzori)) {
//            error = unghie() - y;
//            p = error * kp;
//            sac += error;
//            i = (int) (sac * ki);
//            d = (int) ((error - lasterror) * kd);
//            _C8_9B = p + i + d;
//            lasterror = (int) error;
//            rightFront.setPower(puterea * w - _C8_9B);
//            leftFront.setPower(puterea * w + _C8_9B);
//            leftRear.setPower(puterea * w + _C8_9B);
//            rightRear.setPower(puterea * w - _C8_9B);
//            contor += 1;
//            wait(10);
//        }
//        stopmotor();
//    }
//}
