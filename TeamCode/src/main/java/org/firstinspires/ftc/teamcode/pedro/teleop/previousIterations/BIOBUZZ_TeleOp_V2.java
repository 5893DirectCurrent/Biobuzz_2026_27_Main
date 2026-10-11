package org.firstinspires.ftc.teamcode.pedro.teleop.previousIterations;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.api.PoseFactory;

import org.firstinspires.ftc.teamcode.pedro.misc.Constants;


@TeleOp(name = "BIOBUZZ_TeleOp_Follower_Turret_noAutoAim")
public class BIOBUZZ_TeleOp_V2 extends LinearOpMode {

    // =========================
    // DRIVE
    // =========================

    private static final double DRIVE_SPEED = 1.0;
    private static final double SLOW_SPEED = 0.40;

    // =========================
    // INTAKE / TRANSFER
    // =========================

    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_POWER = 1.0;

    // =========================
    // SHOOTER
    // =========================

    private static final double SHOOTER_LOW = 1200;
    private static final double SHOOTER_MEDIUM = 1500;
    private static final double SHOOTER_HIGH = 1800;

    // Tune these later
    private static final double SHOOTER_P = 0.0;
    private static final double SHOOTER_I = 0.0;
    private static final double SHOOTER_D = 0.0;
    private static final double SHOOTER_F = 0.0;

    // =========================
    // TURRET
    // =========================

    private static final double TURRET_CENTER = 0.50;

    // Start conservatively.
    // Expand these after testing the physical mechanism.
    private static final double TURRET_MIN = 0.20;
    private static final double TURRET_MAX = 0.80;

    private static final double TURRET_SPEED = 0.01;

    // =========================
    // PEDRO PATHING
    // =========================

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // CHANGE THIS to your actual starting position.
    private final Pose startPose = poseFactory.of(24, 24, 0);

    // =========================
    // HARDWARE
    // =========================

    private DcMotor LF;
    private DcMotor RF;
    private DcMotor LB;
    private DcMotor RB;

    private DcMotor intake;
    private DcMotor transfer;

    private DcMotorEx shooter;

    private Servo turret;

    // =========================
    // STATE
    // =========================

    private boolean shooterOn = false;

    private double shooterTarget = SHOOTER_LOW;

    private double turretPosition = TURRET_CENTER;

    private boolean lastCenterButton = false;


    @Override
    public void runOpMode() {

        // =========================
        // HARDWARE MAP
        // =========================

        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");

        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        turret = hardwareMap.get(Servo.class, "turret");


        // =========================
        // MOTOR DIRECTIONS
        // =========================

        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);

        RF.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.FORWARD);


        // =========================
        // ZERO POWER
        // =========================

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        // =========================
        // SHOOTER
        // =========================

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setVelocityPIDFCoefficients(
                SHOOTER_P,
                SHOOTER_I,
                SHOOTER_D,
                SHOOTER_F
        );


        // =========================
        // TURRET
        // =========================

        turretPosition = TURRET_CENTER;
        turret.setPosition(turretPosition);


        // =========================
        // PEDRO PATHING
        // =========================

        follower = Constants.create(hardwareMap);

        follower.setPose(startPose);


        // =========================
        // INIT TELEMETRY
        // =========================

        telemetry.addLine("BIOBUZZ TeleOp V2");
        telemetry.addLine("Pedro Localization Ready");

        telemetry.update();


        waitForStart();


        // =========================
        // MAIN LOOP
        // =========================

        while (opModeIsActive()) {

            // Pedro localization
            follower.update();


            // Drive
            runDrive();


            // Intake
            runIntake();


            // Transfer
            runTransfer();


            // Shooter
            runShooter();


            // Turret
            runTurret();


            // Telemetry
            updateTelemetry();
        }
    }


    // =========================================================
    // DRIVE
    // =========================================================

    private void runDrive() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double denominator =
                Math.max(
                        Math.abs(y) + Math.abs(x) + Math.abs(rx),
                        1.0
                );

        double lfPower =
                (y + x + rx) / denominator;

        double rfPower =
                (y - x - rx) / denominator;

        double lbPower =
                (y - x + rx) / denominator;

        double rbPower =
                (y + x - rx) / denominator;


        double speed = DRIVE_SPEED;

        if (gamepad1.left_bumper) {
            speed = SLOW_SPEED;
        }


        LF.setPower(lfPower * speed);
        RF.setPower(rfPower * speed);
        LB.setPower(lbPower * speed);
        RB.setPower(rbPower * speed);
    }


    // =========================================================
    // INTAKE
    // =========================================================

    private void runIntake() {

        if (gamepad1.right_bumper) {

            intake.setPower(INTAKE_POWER);

        } else if (gamepad1.a) {

            intake.setPower(-INTAKE_POWER);

        } else {

            intake.setPower(0);
        }
    }


    // =========================================================
    // TRANSFER
    // =========================================================

    private void runTransfer() {

        if (gamepad1.right_bumper || gamepad2.b) {

            transfer.setPower(TRANSFER_POWER);

        } else if (gamepad1.a || gamepad2.x) {

            transfer.setPower(-TRANSFER_POWER);

        } else {

            transfer.setPower(0);
        }
    }


    // =========================================================
    // SHOOTER
    // =========================================================

    private void runShooter() {

        if (gamepad2.a) {
            shooterOn = !shooterOn;
        }


        if (gamepad2.dpad_up) {
            shooterTarget = SHOOTER_HIGH;
        }

        if (gamepad2.dpad_left) {
            shooterTarget = SHOOTER_MEDIUM;
        }

        if (gamepad2.dpad_down) {
            shooterTarget = SHOOTER_LOW;
        }


        if (shooterOn) {

            shooter.setVelocity(shooterTarget);

        } else {

            shooter.setVelocity(0);
        }
    }


    // =========================================================
    // TURRET
    // =========================================================

    private void runTurret() {

        // Center turret with right stick button
        if (gamepad2.right_stick_button && !lastCenterButton) {

            turretPosition = TURRET_CENTER;
        }

        lastCenterButton =
                gamepad2.right_stick_button;


        // Manual turret control
        double turretStick =
                gamepad2.right_stick_x;


        if (Math.abs(turretStick) > 0.05) {

            turretPosition +=
                    turretStick * TURRET_SPEED;
        }


        // Software limits
        turretPosition =
                Math.max(
                        TURRET_MIN,
                        Math.min(
                                TURRET_MAX,
                                turretPosition
                        )
                );


        turret.setPosition(turretPosition);
    }


    // =========================================================
    // TELEMETRY
    // =========================================================

    private void updateTelemetry() {

        // =========================
        // PEDRO POSE
        // =========================

        Pose currentPose = follower.pose();

        double robotX = currentPose.x();
        double robotY = currentPose.y();
        double robotHeading = currentPose.heading();


        telemetry.addLine("===== PEDRO LOCALIZATION =====");

        telemetry.addData(
                "Robot X",
                robotX
        );

        telemetry.addData(
                "Robot Y",
                robotY
        );

        telemetry.addData(
                "Heading",
                Math.toDegrees(robotHeading)
        );


        // =========================
        // SHOOTER
        // =========================

        telemetry.addLine("===== SHOOTER =====");

        telemetry.addData(
                "Shooter On",
                shooterOn
        );

        telemetry.addData(
                "Target Velocity",
                shooterTarget
        );

        telemetry.addData(
                "Actual Velocity",
                shooter.getVelocity()
        );

        telemetry.addData(
                "Velocity Error",
                shooterTarget - shooter.getVelocity()
        );


        // =========================
        // TURRET
        // =========================

        telemetry.addLine("===== TURRET =====");

        telemetry.addData(
                "Turret Position",
                turretPosition
        );

        telemetry.addData(
                "Turret Center",
                TURRET_CENTER
        );

        telemetry.addData(
                "Turret Min",
                TURRET_MIN
        );

        telemetry.addData(
                "Turret Max",
                TURRET_MAX
        );


        telemetry.update();
    }
}
