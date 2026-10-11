package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedro.misc.Constants;

/*
 * BIOBUZZ teleop with a turret aimed from the Pinpoint pose.
 *
 * Drive, intake, transfer, and shooter match BIOBUZZ TeleOp V1.
 * The Pinpoint is read only for aiming, so it does not take over the drive motors.
 *
 * Add a motor named "turret" to the hardware map.
 * Point that turret forward and press Driver 2 Y before auto-aim.
 * Set the robot on the start pose, then press Driver 1 Back to reset odometry.
 *
 * Field frame: center of the field, +X toward the audience, +Y toward blue.
 * Heading 0 points toward the audience. The turret aims at the upward HIVE cell.
 */

@TeleOp(name = "BIOBUZZ Odometry Turret", group = "BIOBUZZ")
public class Odometry_Based_Turret_Test extends LinearOpMode {

    // ============================================================
    // DRIVE
    // ============================================================

    public static double DRIVE_SPEED = 1.0;
    public static double SLOW_SPEED = 0.40;

    // ============================================================
    // INTAKE / TRANSFER
    // ============================================================

    public static double INTAKE_POWER = 1.0;
    public static double TRANSFER_POWER = 1.0;

    // ============================================================
    // SHOOTER
    // ============================================================

    public static double SHOOTER_P = 0.0000;
    public static double SHOOTER_I = 0.0000;
    public static double SHOOTER_D = 0.0000;
    public static double SHOOTER_F = 0.0000;

    public static double SHOOTER_LOW = 1200;
    public static double SHOOTER_MEDIUM = 1500;
    public static double SHOOTER_HIGH = 1800;

    // ============================================================
    // FIELD
    // Inches. Origin is the center of the field, on top of the tiles.
    // +X is toward the audience. +Y is toward the blue wall.
    // Heading 0 points toward the audience.
    //
    // Launch targets are the upward CELL openings from the field CAD.
    // At match start the red audience cell is up and the blue scoring
    // cell is up. Driver 2 dpad-right swaps the cell after a HIVE TIP.
    // ============================================================

    public static double START_X = 0.0;
    public static double START_Y = 0.0;
    public static double START_HEADING_DEGREES = 0.0;

    public static double RED_AUDIENCE_CELL_X = 7.57;
    public static double RED_AUDIENCE_CELL_Y = -12.74;
    public static double RED_SCORING_CELL_X = -7.57;
    public static double RED_SCORING_CELL_Y = -12.74;

    public static double BLUE_AUDIENCE_CELL_X = 7.57;
    public static double BLUE_AUDIENCE_CELL_Y = 12.76;
    public static double BLUE_SCORING_CELL_X = -7.57;
    public static double BLUE_SCORING_CELL_Y = 12.76;

    // Turret pivot measured from the pinpoint tracking point.
    // +X is robot forward, +Y is robot left.
    public static double TURRET_OFFSET_X = 0.0;
    public static double TURRET_OFFSET_Y = 0.0;

    // ============================================================
    // TURRET
    // goBILDA 435 RPM is about 384.5 ticks per output revolution.
    // Multiply by any gear after the motor.
    // ============================================================

    public static double TICKS_PER_REVOLUTION = 384.5;
    public static double TURRET_ZERO_OFFSET_DEGREES = 0.0;
    public static double TURRET_TICK_SIGN = 1.0;
    public static double TURRET_MIN_DEGREES = -180.0;
    public static double TURRET_MAX_DEGREES = 180.0;
    public static double TURRET_POWER = 0.6;
    public static double MANUAL_TURRET_POWER = 0.3;

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
    private DcMotorEx turret;

    private PinpointLocalizer pinpoint;
    private final PoseFactory degrees = PoseFactory.degrees();

    private boolean shooterOn = false;
    private double shooterTarget = 0;
    private boolean lastShooterButton = false;

    private boolean blueAlliance = true;
    private boolean autoAim = false;

    // Section 11.1 of the field setup guide: red audience cell starts up,
    // blue scoring cell starts up.
    private boolean redAudienceCellUp = true;
    private boolean blueAudienceCellUp = false;

    private boolean lastAimButton = false;
    private boolean lastAllianceButton = false;
    private boolean lastZeroButton = false;
    private boolean lastPoseButton = false;
    private boolean lastCellButton = false;

    @Override
    public void runOpMode() {

        LF = hardwareMap.get(DcMotorEx.class, "LF");
        RF = hardwareMap.get(DcMotorEx.class, "RF");
        LB = hardwareMap.get(DcMotorEx.class, "LB");
        RB = hardwareMap.get(DcMotorEx.class, "RB");

        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);

        zeroTurretEncoder();

        pinpoint = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        resetFieldPose();

        telemetry.addLine("BIOBUZZ ODOMETRY TURRET");
        telemetry.addLine("Driver 1: drive and collect");
        telemetry.addLine("Driver 2: shooter, transfer, turret");
        telemetry.addLine("Driver 2 Y zeros the turret. Point it forward first.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            driveRobot();
            runIntake();
            runTransfer();
            runShooter();

            pinpoint.update();
            readTurretButtons();
            aimTurret();
            showTelemetry();
        }

        turret.setPower(0);
        shooter.setVelocity(0);
    }

    // ============================================================
    // DRIVE
    // Same mix and directions as BIOBUZZ TeleOp V1.
    // ============================================================

    private void driveRobot() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        double speed = gamepad1.left_bumper ? SLOW_SPEED : DRIVE_SPEED;

        double frontLeft = y + x + turn;
        double frontRight = y - x - turn;
        double backLeft = y - x + turn;
        double backRight = y + x - turn;

        double max = Math.max(1.0, Math.max(
                Math.abs(frontLeft),
                Math.max(Math.abs(frontRight), Math.max(Math.abs(backLeft), Math.abs(backRight)))
        ));

        LF.setPower(frontLeft / max * speed);
        RF.setPower(frontRight / max * speed);
        LB.setPower(backLeft / max * speed);
        RB.setPower(backRight / max * speed);
    }

    private void runIntake() {

        if (gamepad1.right_bumper) {
            intake.setPower(INTAKE_POWER);
            transfer.setPower(TRANSFER_POWER);
        } else if (gamepad1.a) {
            intake.setPower(-INTAKE_POWER);
            transfer.setPower(-TRANSFER_POWER);
        } else {
            intake.setPower(0);
        }
    }

    private void runTransfer() {

        if (gamepad2.b) {
            transfer.setPower(TRANSFER_POWER);
        } else if (gamepad2.x) {
            transfer.setPower(-TRANSFER_POWER);
        } else if (!gamepad1.right_bumper && !gamepad1.a) {
            transfer.setPower(0);
        }
    }

    private void runShooter() {

        if (gamepad2.a && !lastShooterButton) {
            shooterOn = !shooterOn;
        }
        lastShooterButton = gamepad2.a;

        if (gamepad2.dpad_up) {
            shooterTarget = SHOOTER_HIGH;
        } else if (gamepad2.dpad_left) {
            shooterTarget = SHOOTER_MEDIUM;
        } else if (gamepad2.dpad_down) {
            shooterTarget = SHOOTER_LOW;
        }

        if (shooterOn && shooterTarget > 0) {
            shooter.setVelocity(shooterTarget);
        } else {
            shooter.setVelocity(0);
        }
    }

    // ============================================================
    // TURRET
    // Driver 2 right bumper: auto-aim
    // Driver 2 left bumper: blue / red hive
    // Driver 2 Y: zero encoder
    // Driver 2 right stick X: manual aim while auto-aim is off
    // Driver 2 dpad right: swap to the other cell after a hive tip
    // Driver 1 Back: reset the field pose
    // ============================================================

    private void readTurretButtons() {

        if (gamepad2.right_bumper && !lastAimButton) {
            autoAim = !autoAim;
        }
        if (gamepad2.left_bumper && !lastAllianceButton) {
            blueAlliance = !blueAlliance;
        }
        if (gamepad2.y && !lastZeroButton) {
            zeroTurretEncoder();
        }
        if (gamepad1.back && !lastPoseButton) {
            resetFieldPose();
        }
        if (gamepad2.dpad_right && !lastCellButton) {
            if (blueAlliance) {
                blueAudienceCellUp = !blueAudienceCellUp;
            } else {
                redAudienceCellUp = !redAudienceCellUp;
            }
        }

        lastAimButton = gamepad2.right_bumper;
        lastAllianceButton = gamepad2.left_bumper;
        lastZeroButton = gamepad2.y;
        lastPoseButton = gamepad1.back;
        lastCellButton = gamepad2.dpad_right;
    }

    private void aimTurret() {

        if (!autoAim) {
            if (turret.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            double power = 0.0;
            if (Math.abs(gamepad2.right_stick_x) > 0.05) {
                power = gamepad2.right_stick_x * MANUAL_TURRET_POWER;
            }
            turret.setPower(power);
            return;
        }

        if (turret.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            turret.setTargetPosition(turret.getCurrentPosition());
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        double turretDegrees = clamp(
                Math.toDegrees(robotRelativeAngle()),
                TURRET_MIN_DEGREES,
                TURRET_MAX_DEGREES
        );

        turret.setTargetPosition(degreesToTicks(turretDegrees));
        turret.setPower(TURRET_POWER);
    }

    private double robotRelativeAngle() {

        Pose pose = pinpoint.state().pose();
        double heading = pose.heading();

        double turretX = pose.x()
                + TURRET_OFFSET_X * Math.cos(heading)
                - TURRET_OFFSET_Y * Math.sin(heading);
        double turretY = pose.y()
                + TURRET_OFFSET_X * Math.sin(heading)
                + TURRET_OFFSET_Y * Math.cos(heading);

        double[] goal = activeCell();
        double fieldAngle = Math.atan2(goal[1] - turretY, goal[0] - turretX);
        double offset = Math.toRadians(TURRET_ZERO_OFFSET_DEGREES);

        return Angle.normalizeSigned(fieldAngle - heading - offset);
    }

    private int degreesToTicks(double turretDegrees) {
        double revolutions = turretDegrees / 360.0;
        return (int) Math.round(revolutions * TICKS_PER_REVOLUTION * TURRET_TICK_SIGN);
    }

    private void zeroTurretEncoder() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(autoAim
                ? DcMotor.RunMode.RUN_TO_POSITION
                : DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setPower(0);
    }

    private void resetFieldPose() {
        pinpoint.setPose(degrees.of(START_X, START_Y, START_HEADING_DEGREES));
    }

    private void showTelemetry() {

        Pose pose = pinpoint.state().pose();
        double[] goal = activeCell();
        boolean audienceUp = blueAlliance ? blueAudienceCellUp : redAudienceCellUp;

        telemetry.addLine("===== BIOBUZZ TURRET =====");
        telemetry.addData("Alliance", blueAlliance ? "BLUE" : "RED");
        telemetry.addData("Cell", audienceUp ? "AUDIENCE" : "SCORING");
        telemetry.addData("Auto aim", autoAim ? "ON" : "OFF");
        telemetry.addData("X", pose.x());
        telemetry.addData("Y", pose.y());
        telemetry.addData("Heading", Math.toDegrees(pose.heading()));
        telemetry.addData("Distance", Math.hypot(goal[0] - pose.x(), goal[1] - pose.y()));
        telemetry.addData("Turret target", Math.toDegrees(robotRelativeAngle()));
        telemetry.addData("Turret ticks", turret.getCurrentPosition());
        telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
        telemetry.addData("Shooter velocity", shooter.getVelocity());
        telemetry.update();
    }

    private double[] activeCell() {
        if (blueAlliance) {
            if (blueAudienceCellUp) {
                return new double[] {BLUE_AUDIENCE_CELL_X, BLUE_AUDIENCE_CELL_Y};
            }
            return new double[] {BLUE_SCORING_CELL_X, BLUE_SCORING_CELL_Y};
        }
        if (redAudienceCellUp) {
            return new double[] {RED_AUDIENCE_CELL_X, RED_AUDIENCE_CELL_Y};
        }
        return new double[] {RED_SCORING_CELL_X, RED_SCORING_CELL_Y};
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}