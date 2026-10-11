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


@TeleOp(name = "BIOBUZZ_TeleOp_RED")
public class BIOBUZZ_TeleOp_RED_turret extends LinearOpMode {

    // =========================================================
    // DRIVE
    // =========================================================

    private static final double DRIVE_SPEED = 1.0;
    private static final double SLOW_SPEED = 0.40;


    // =========================================================
    // INTAKE / TRANSFER
    // =========================================================

    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_POWER = 1.0;


    // =========================================================
    // SHOOTER
    // =========================================================

    private static final double SHOOTER_LOW = 1200;
    private static final double SHOOTER_MEDIUM = 1500;
    private static final double SHOOTER_HIGH = 1800;

    // Tune these later
    private static final double SHOOTER_P = 0.0;
    private static final double SHOOTER_I = 0.0;
    private static final double SHOOTER_D = 0.0;
    private static final double SHOOTER_F = 0.0;


    // =========================================================
    // TURRET
    // =========================================================

    private static final double TURRET_CENTER = 0.50;

    // Conservative limits for initial testing
    private static final double TURRET_MIN = 0.20;
    private static final double TURRET_MAX = 0.80;

    private static final double TURRET_SPEED = 0.01;

    // Change to -1.0 if auto aim turns the wrong direction
    private static final double TURRET_DIRECTION = 1.0;


    // =========================================================
    // RED HIVE
    // =========================================================

    private static final double RED_HIVE_X = 72.0;
    private static final double RED_HIVE_Y = 84.75;


    // =========================================================
    // PEDRO PATHING
    // =========================================================

    private Follower follower;

    private final PoseFactory poseFactory =
            PoseFactory.degrees();

    // TEMPORARY
    // Change this to your actual red-side starting pose
    private final Pose startPose =
            poseFactory.of(24, 24, 0);


    // =========================================================
    // HARDWARE
    // =========================================================

    private DcMotor LF;
    private DcMotor RF;
    private DcMotor LB;
    private DcMotor RB;

    private DcMotor intake;
    private DcMotor transfer;

    private DcMotorEx shooter;

    private Servo turret;


    // =========================================================
    // STATE
    // =========================================================

    private boolean shooterOn = false;

    private double shooterTarget = SHOOTER_LOW;

    private boolean lastShooterButton = false;


    private double turretPosition = TURRET_CENTER;

    private boolean autoAim = false;

    private boolean lastAutoAimButton = false;
    private boolean lastCenterButton = false;

    private double turretAngle = 0.0;
    private double angleToHive = 0.0;


    @Override
    public void runOpMode() {

        // =====================================================
        // HARDWARE MAP
        // =====================================================

        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");

        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        turret = hardwareMap.get(Servo.class, "turret");


        // =====================================================
        // MOTOR DIRECTIONS
        // =====================================================

        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);

        RF.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.FORWARD);


        // =====================================================
        // ZERO POWER
        // =====================================================

        LF.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        RF.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        LB.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        RB.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        intake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        transfer.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        shooter.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );


        // =====================================================
        // SHOOTER SETUP
        // =====================================================

        shooter.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        shooter.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        shooter.setVelocityPIDFCoefficients(
                SHOOTER_P,
                SHOOTER_I,
                SHOOTER_D,
                SHOOTER_F
        );


        // =====================================================
        // TURRET SETUP
        // =====================================================

        turretPosition = TURRET_CENTER;

        turret.setPosition(
                turretPosition
        );


        // =====================================================
        // PEDRO PATHING
        // =====================================================

        follower =
                Constants.create(hardwareMap);

        follower.setPose(
                startPose
        );


        // =====================================================
        // READY
        // =====================================================

        telemetry.addLine(
                "BIOBUZZ RED TeleOp"
        );

        telemetry.addLine(
                "Pedro Localization Ready"
        );

        telemetry.addLine(
                "RED HIVE Auto Aim Ready"
        );

        telemetry.update();


        waitForStart();


        // =====================================================
        // MAIN LOOP
        // =====================================================

        while (opModeIsActive()) {

            // Update localization first
            follower.update();

            runDrive();

            runIntake();

            runTransfer();

            runShooter();

            runTurret();

            updateTelemetry();
        }
    }


    // =========================================================
    // DRIVE
    // =========================================================

    private void runDrive() {

        double y =
                -gamepad1.left_stick_y;

        double x =
                gamepad1.left_stick_x;

        double rx =
                gamepad1.right_stick_x;


        double denominator =
                Math.max(
                        Math.abs(y)
                                + Math.abs(x)
                                + Math.abs(rx),
                        1.0
                );


        double lfPower =
                (y + x + rx)
                        / denominator;

        double rfPower =
                (y - x - rx)
                        / denominator;

        double lbPower =
                (y - x + rx)
                        / denominator;

        double rbPower =
                (y + x - rx)
                        / denominator;


        double speed =
                DRIVE_SPEED;


        if (gamepad1.left_bumper) {

            speed =
                    SLOW_SPEED;
        }


        LF.setPower(
                lfPower * speed
        );

        RF.setPower(
                rfPower * speed
        );

        LB.setPower(
                lbPower * speed
        );

        RB.setPower(
                rbPower * speed
        );
    }


    // =========================================================
    // INTAKE
    // =========================================================

    private void runIntake() {

        if (gamepad1.right_bumper) {

            intake.setPower(
                    INTAKE_POWER
            );

        } else if (gamepad1.a) {

            intake.setPower(
                    -INTAKE_POWER
            );

        } else {

            intake.setPower(0);
        }
    }


    // =========================================================
    // TRANSFER
    // =========================================================

    private void runTransfer() {

        if (gamepad1.right_bumper
                || gamepad2.b) {

            transfer.setPower(
                    TRANSFER_POWER
            );

        } else if (gamepad1.a
                || gamepad2.x) {

            transfer.setPower(
                    -TRANSFER_POWER
            );

        } else {

            transfer.setPower(0);
        }
    }


    // =========================================================
    // SHOOTER
    // =========================================================

    private void runShooter() {

        // Toggle shooter
        if (gamepad2.a
                && !lastShooterButton) {

            shooterOn =
                    !shooterOn;
        }

        lastShooterButton =
                gamepad2.a;


        // High
        if (gamepad2.dpad_up) {

            shooterTarget =
                    SHOOTER_HIGH;
        }


        // Medium
        if (gamepad2.dpad_left) {

            shooterTarget =
                    SHOOTER_MEDIUM;
        }


        // Low
        if (gamepad2.dpad_down) {

            shooterTarget =
                    SHOOTER_LOW;
        }


        if (shooterOn) {

            shooter.setVelocity(
                    shooterTarget
            );

        } else {

            shooter.setVelocity(0);
        }
    }


    // =========================================================
    // TURRET
    // =========================================================

    private void runTurret() {

        // -----------------------------------------------------
        // AUTO AIM TOGGLE
        // Left stick button
        // -----------------------------------------------------

        if (gamepad2.left_stick_button
                && !lastAutoAimButton) {

            autoAim =
                    !autoAim;
        }

        lastAutoAimButton =
                gamepad2.left_stick_button;


        // -----------------------------------------------------
        // CENTER TURRET
        // Right stick button
        // -----------------------------------------------------

        if (gamepad2.right_stick_button
                && !lastCenterButton) {

            autoAim = false;

            turretPosition =
                    TURRET_CENTER;
        }

        lastCenterButton =
                gamepad2.right_stick_button;


        // -----------------------------------------------------
        // MANUAL TURRET
        // -----------------------------------------------------

        double turretStick =
                gamepad2.right_stick_x;


        if (Math.abs(turretStick) > 0.05) {

            // Manual movement overrides auto aim
            autoAim = false;

            turretPosition +=
                    turretStick
                            * TURRET_SPEED;
        }


        // -----------------------------------------------------
        // RED HIVE AUTO AIM
        // -----------------------------------------------------

        if (autoAim) {

            Pose currentPose =
                    follower.pose();


            double robotX =
                    currentPose.x();

            double robotY =
                    currentPose.y();

            double robotHeading =
                    Math.toDegrees(
                            currentPose.heading()
                    );


            // Direction from robot to red HIVE
            angleToHive =
                    Math.toDegrees(
                            Math.atan2(
                                    RED_HIVE_Y
                                            - robotY,

                                    RED_HIVE_X
                                            - robotX
                            )
                    );


            // Convert field angle into
            // robot-relative turret angle
            turretAngle =
                    angleToHive
                            - robotHeading;


            // Normalize angle
            while (turretAngle > 180.0) {

                turretAngle -=
                        360.0;
            }


            while (turretAngle < -180.0) {

                turretAngle +=
                        360.0;
            }


            // Turret can physically rotate
            // 90 degrees each direction
            turretAngle =
                    Math.max(
                            -90.0,
                            Math.min(
                                    90.0,
                                    turretAngle
                            )
                    );


            // 1:1 angle conversion
            //
            // -90 deg = 0.0
            //   0 deg = 0.5
            // +90 deg = 1.0

            turretPosition =
                    TURRET_CENTER
                            + TURRET_DIRECTION
                            * (turretAngle / 180.0);
        }


        // -----------------------------------------------------
        // SOFTWARE LIMITS
        // -----------------------------------------------------

        turretPosition =
                Math.max(
                        TURRET_MIN,
                        Math.min(
                                TURRET_MAX,
                                turretPosition
                        )
                );


        turret.setPosition(
                turretPosition
        );
    }


    // =========================================================
    // TELEMETRY
    // =========================================================

    private void updateTelemetry() {

        Pose currentPose =
                follower.pose();


        double robotX =
                currentPose.x();

        double robotY =
                currentPose.y();

        double robotHeading =
                Math.toDegrees(
                        currentPose.heading()
                );


        // -----------------------------------------------------
        // PEDRO
        // -----------------------------------------------------

        telemetry.addLine(
                "===== PEDRO ====="
        );

        telemetry.addData(
                "Robot X",
                "%.2f",
                robotX
        );

        telemetry.addData(
                "Robot Y",
                "%.2f",
                robotY
        );

        telemetry.addData(
                "Heading",
                "%.2f deg",
                robotHeading
        );


        // -----------------------------------------------------
        // AUTO AIM
        // -----------------------------------------------------

        telemetry.addLine(
                "===== AUTO AIM ====="
        );

        telemetry.addData(
                "Auto Aim",
                autoAim
                        ? "ON"
                        : "OFF"
        );

        telemetry.addData(
                "Target",
                "RED HIVE"
        );

        telemetry.addData(
                "Target X",
                "%.2f",
                RED_HIVE_X
        );

        telemetry.addData(
                "Target Y",
                "%.2f",
                RED_HIVE_Y
        );

        telemetry.addData(
                "Angle To Hive",
                "%.2f deg",
                angleToHive
        );

        telemetry.addData(
                "Turret Angle",
                "%.2f deg",
                turretAngle
        );

        telemetry.addData(
                "Turret Position",
                "%.3f",
                turretPosition
        );


        // -----------------------------------------------------
        // SHOOTER
        // -----------------------------------------------------

        telemetry.addLine(
                "===== SHOOTER ====="
        );

        telemetry.addData(
                "Shooter",
                shooterOn
                        ? "ON"
                        : "OFF"
        );

        telemetry.addData(
                "Target Velocity",
                "%.0f",
                shooterTarget
        );

        telemetry.addData(
                "Actual Velocity",
                "%.0f",
                shooter.getVelocity()
        );

        telemetry.addData(
                "Velocity Error",
                "%.0f",
                shooterTarget
                        - shooter.getVelocity()
        );


        telemetry.update();
    }
}