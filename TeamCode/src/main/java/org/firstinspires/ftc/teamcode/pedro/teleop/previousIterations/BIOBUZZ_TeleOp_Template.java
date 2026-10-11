package org.firstinspires.ftc.teamcode.pedro.teleop.previousIterations;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "BIOBUZZ TeleOp V1", group = "BIOBUZZ")
public class BIOBUZZ_TeleOp_Template extends LinearOpMode {

    // ============================================================
    // SETTINGS
    // ============================================================

    // ---------------- DRIVE ----------------

    public static double DRIVE_SPEED = 1.0;
    public static double SLOW_SPEED = 0.40;

    // ---------------- INTAKE ----------------

    public static double INTAKE_POWER = 1.0;

    // ---------------- TRANSFER ----------------

    public static double TRANSFER_POWER = 1.0;

    // ---------------- SHOOTER ----------------

    // Change these while tuning
    public static double SHOOTER_P = 0.0000;
    public static double SHOOTER_I = 0.0000;
    public static double SHOOTER_D = 0.0000;
    public static double SHOOTER_F = 0.0000;

    // Shooter velocity presets
    public static double SHOOTER_LOW = 1200;
    public static double SHOOTER_MEDIUM = 1500;
    public static double SHOOTER_HIGH = 1800;

    // ============================================================
    // HARDWARE
    // ============================================================

    private DcMotorEx LF;
    private DcMotorEx RF;
    private DcMotorEx LB;
    private DcMotorEx RB;

    private DcMotor intake;
    private DcMotor transfer;

    private DcMotorEx shooter;

    // ============================================================
    // VARIABLES
    // ============================================================

    private boolean shooterOn = false;

    private double shooterTarget = 0;

    private boolean lastShooterButton = false;

    @Override
    public void runOpMode() {

        // ========================================================
        // CONNECT TO HARDWARE
        // ========================================================

        LF = hardwareMap.get(DcMotorEx.class, "LF");
        RF = hardwareMap.get(DcMotorEx.class, "RF");
        LB = hardwareMap.get(DcMotorEx.class, "LB");
        RB = hardwareMap.get(DcMotorEx.class, "RB");

        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        // ========================================================
        // MOTOR DIRECTIONS
        // ========================================================

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);

        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        // ========================================================
        // BRAKE SETTINGS
        // ========================================================

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // ========================================================
        // SHOOTER ENCODER
        // ========================================================

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ========================================================
        // SHOOTER PIDF
        // ========================================================

        shooter.setVelocityPIDFCoefficients(
                SHOOTER_P,
                SHOOTER_I,
                SHOOTER_D,
                SHOOTER_F
        );

        // ========================================================
        // READY
        // ========================================================

        telemetry.addLine("BIOBUZZ TELEOP V1");
        telemetry.addLine("----------------");
        telemetry.addLine("Driver 1: Drive + Collect");
        telemetry.addLine("Driver 2: Shooter + Score");
        telemetry.addLine("");
        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        // ========================================================
        // MAIN TELEOP LOOP
        // ========================================================

        while (opModeIsActive()) {

            // ----------------------------------------------------
            // DRIVER 1
            // ----------------------------------------------------

            driveRobot();

            runIntake();

            // ----------------------------------------------------
            // DRIVER 2
            // ----------------------------------------------------

            runShooter();

            runTransfer();

            // ----------------------------------------------------
            // TELEMETRY
            // ----------------------------------------------------

            showTelemetry();
        }
    }

    // ============================================================
    // DRIVE
    // ============================================================

    private void driveRobot() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double speed = DRIVE_SPEED;

        // Left bumper = slow mode
        if (gamepad1.left_bumper) {
            speed = SLOW_SPEED;
        }

        // Mecanum calculations

        double frontLeft = y + x + turn;
        double frontRight = y - x - turn;

        double backLeft = y - x + turn;
        double backRight = y + x - turn;

        // Normalize powers

        double max = Math.max(
                1.0,
                Math.max(
                        Math.abs(frontLeft),
                        Math.max(
                                Math.abs(frontRight),
                                Math.max(
                                        Math.abs(backLeft),
                                        Math.abs(backRight)
                                )
                        )
                )
        );

        frontLeft /= max;
        frontRight /= max;
        backLeft /= max;
        backRight /= max;

        // Apply power

        LF.setPower(frontLeft * speed);
        RF.setPower(frontRight * speed);
        LB.setPower(backLeft * speed);
        RB.setPower(backRight * speed);
    }

    // ============================================================
    // INTAKE
    // ============================================================

    private void runIntake() {

        // Right bumper = collect
        if (gamepad1.right_bumper) {

            intake.setPower(INTAKE_POWER);

            // Intake and transfer work together
            transfer.setPower(TRANSFER_POWER);

        }

        // A = reverse / unjam
        else if (gamepad1.a) {

            intake.setPower(-INTAKE_POWER);
            transfer.setPower(-TRANSFER_POWER);

        }

        else {

            intake.setPower(0);

            // Do not automatically stop transfer here.
            // Driver 2 can still control it.

        }
    }

    // ============================================================
    // TRANSFER
    // ============================================================

    private void runTransfer() {

        // Driver 1 is currently using the intake + transfer
        // together for collecting.

        // Driver 2 can manually control the transfer.

        if (gamepad2.b) {

            transfer.setPower(TRANSFER_POWER);

        }

        else if (gamepad2.x) {

            transfer.setPower(-TRANSFER_POWER);

        }

        // Stop transfer only if Driver 1 is not collecting
        // and Driver 2 isn't manually using it.

        else if (!gamepad1.right_bumper && !gamepad1.a) {

            transfer.setPower(0);

        }
    }

    // ============================================================
    // SHOOTER
    // ============================================================

    private void runShooter() {

        // A = shooter ON/OFF
        boolean shooterButton = gamepad2.a;

        // Detect button press instead of holding the button
        if (shooterButton && !lastShooterButton) {

            shooterOn = !shooterOn;
        }

        lastShooterButton = shooterButton;

        // Shooter velocity selection

        if (gamepad2.dpad_up) {

            shooterTarget = SHOOTER_HIGH;

        }

        else if (gamepad2.dpad_left) {

            shooterTarget = SHOOTER_MEDIUM;

        }

        else if (gamepad2.dpad_down) {

            shooterTarget = SHOOTER_LOW;
        }

        // Run shooter

        if (shooterOn && shooterTarget > 0) {

            shooter.setVelocity(shooterTarget);

        }

        else {

            shooter.setVelocity(0);
        }
    }

    // ============================================================
    // TELEMETRY
    // ============================================================

    private void showTelemetry() {

        double actualVelocity = shooter.getVelocity();

        double velocityError = shooterTarget - actualVelocity;

        telemetry.addLine("===== BIOBUZZ TELEOP =====");

        telemetry.addLine("");

        telemetry.addLine("DRIVER 1");
        telemetry.addData(
                "Slow Mode",
                gamepad1.left_bumper ? "ON" : "OFF"
        );

        telemetry.addData(
                "Collecting",
                gamepad1.right_bumper ? "YES" : "NO"
        );

        telemetry.addLine("");

        telemetry.addLine("DRIVER 2");

        telemetry.addData(
                "Shooter",
                shooterOn ? "ON" : "OFF"
        );

        telemetry.addData(
                "Target Velocity",
                "%.0f",
                shooterTarget
        );

        telemetry.addData(
                "Actual Velocity",
                "%.0f",
                actualVelocity
        );

        telemetry.addData(
                "Velocity Error",
                "%.0f",
                velocityError
        );

        telemetry.addLine("");

        telemetry.addLine("PIDF");

        telemetry.addData("P", SHOOTER_P);
        telemetry.addData("I", SHOOTER_I);
        telemetry.addData("D", SHOOTER_D);
        telemetry.addData("F", SHOOTER_F);

        telemetry.update();
    }
}