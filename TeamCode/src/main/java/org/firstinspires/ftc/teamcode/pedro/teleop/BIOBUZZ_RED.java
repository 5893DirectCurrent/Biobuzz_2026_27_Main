package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import org.firstinspires.ftc.teamcode.pedro.misc.Constants;

/** Standalone BIOBUZZ RED OpMode; helper classes are file-local by unique name. */
@TeleOp(name = "BIOBUZZ RED")
public class BIOBUZZ_RED extends BiobuzzMatch_RED {
    @Override protected double referenceX() { return 58.0; }
    @Override protected String allianceName() { return "RED"; }
}


/** Shared match logic. RED and BLUE only supply the reference X. */
abstract class BiobuzzMatch_RED extends LinearOpMode {
    protected abstract double referenceX();
    protected abstract String allianceName();
    @Override public void runOpMode() {
        Scheduler.reset();
        RobotParts_RED hw = new RobotParts_RED(hardwareMap);
        try {
            DrivePart_RED drive = new DrivePart_RED(hw);
            IntakePart_RED intake = new IntakePart_RED(hw);
            TransferPart_RED transfer = new TransferPart_RED(hw);
            ShooterPart_RED shooter = new ShooterPart_RED(hw);
            TurretPart_RED turret = new TurretPart_RED(hw);
            Follower follower = Constants.create(hardwareMap);

            /* TEMPORARY POSE: set the actual field starting position before
             * enabling auto aim. This is not absolute localization.
             * FUTURE LIMELIGHT: in INIT, obtain a validated AprilTag field pose,
             * transform units/axes/heading into Pedro's frame, then setPose.
             * FUTURE LIMELIGHT: about every 30 s, accept a new measurement only
             * after validity, timestamp, and jump checks; otherwise keep Pedro.
             */
            follower.setPose(PoseFactory.degrees().of(
                    RobotTuning_RED.START_X, RobotTuning_RED.START_Y, RobotTuning_RED.START_HEADING_DEG));

            Command driveCmd = BiobuzzIvy_RED.drive(drive, gamepad1);
            Command intakeCmd = BiobuzzIvy_RED.intake(intake, gamepad1);
            Command transferCmd = BiobuzzIvy_RED.transfer(transfer, gamepad1, gamepad2);
            Command shooterCmd = BiobuzzIvy_RED.shooter(shooter, gamepad2);
            Command manualCmd = BiobuzzIvy_RED.manualTurret(turret, gamepad2);
            Command autoCmd = BiobuzzIvy_RED.autoTurret(turret, follower, referenceX());
            TurretSwitch_RED turretSwitch = new TurretSwitch_RED(turret, manualCmd, autoCmd);

            telemetry.addData("Alliance", allianceName());
            telemetry.addLine("L3 auto aim | R3 center | right stick manual override");
            telemetry.addLine("WARNING: fixed starting pose until AprilTag mapping ready");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;
            schedule(driveCmd, intakeCmd, transferCmd, shooterCmd, manualCmd);
            while (opModeIsActive()) {
                follower.update();
                turretSwitch.update(gamepad2);
                Scheduler.execute();
                Pose pose = follower.pose();
                telemetry.addData("Alliance", allianceName());
                telemetry.addData("Turret", turretSwitch.isAuto() ? "AUTO" : "MANUAL");
                telemetry.addData("Aim limit", turret.isLimited() ? "LIMITED" : "OK");
                telemetry.addData("Shooter", shooter.isEnabled() ? "ON" : "OFF");
                telemetry.addData("Shooter target", "%.0f ticks/s", shooter.target());
                telemetry.addData("Shooter left", "%.0f ticks/s", shooter.actualLeft());
                telemetry.addData("Shooter right", "%.0f ticks/s", shooter.actualRight());
                telemetry.addData("Pose", "%.1f, %.1f, %.1f deg",
                        pose.x(), pose.y(), Math.toDegrees(pose.heading()));
                telemetry.addData("Localization", "FIXED START / PEDRO");
                telemetry.update();
            }
        } finally {
            Scheduler.reset();
            hw.stopInitialized();
        }
    }
}

/** All numbers that will need robot-side calibration. */
final class RobotTuning_RED {
    private RobotTuning_RED() {}
    static final double START_X = 24, START_Y = 24, START_HEADING_DEG = 0;
    static final double DRIVE_SPEED = 1, SLOW_SPEED = .40;
    static final double INTAKE_POWER = 1, TRANSFER_POWER = 1;
    static final double SHOOTER_LOW = 1200, SHOOTER_MED = 1500, SHOOTER_HIGH = 1800;
    static final double SHOOTER_P = 0, SHOOTER_I = 0, SHOOTER_D = 0, SHOOTER_F = 0;
    static final double TURRET_CENTER = .50, TURRET_MIN = .20, TURRET_MAX = .80;
    static final double TURRET_SPEED = .01, TURRET_DIRECTION = 1;
    static final double TURRET_DEGREES_PER_SERVO_UNIT = 180;
    static final double TURRET_MAX_ANGLE_DEG = 90;
    static final double TURRET_OFFSET_X = 0, TURRET_OFFSET_Y = 0;
    static final double TURRET_MOUNT_ZERO_DEG = 0;
    static final double TARGET_Y = 72, TARGET_Z = 0; /* Z reserved for shooter geometry. */
}

/** Lazy hardware initialization: TEST loads only its selected mechanism. */
final class RobotParts_RED {
    final HardwareMap map;
    DcMotor lf, rf, lb, rb, intake, transfer;
    DcMotorEx shooterLeft, shooterRight;
    Servo turret;
    RobotParts_RED(HardwareMap map) { this.map = map; }
    void initDrive() {
        lf = map.get(DcMotor.class, "LF"); rf = map.get(DcMotor.class, "RF");
        lb = map.get(DcMotor.class, "LB"); rb = map.get(DcMotor.class, "RB");
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);
        for (DcMotor m : new DcMotor[]{lf, rf, lb, rb}) m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    void initIntake() {
        intake = map.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    void initTransfer() {
        transfer = map.get(DcMotor.class, "transfer");
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    void initShooter() {
        /* Configure both motors for the same overhand shooter mechanism.
         * Hardware names and physical directions must be verified on the robot.
         * Reverse one motor in configuration if its mounting requires it.
         */
        shooterLeft = map.get(DcMotorEx.class, "shooterLeft");
        shooterRight = map.get(DcMotorEx.class, "shooterRight");
        shooterLeft.setDirection(DcMotor.Direction.FORWARD);
        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        for (DcMotorEx motor : new DcMotorEx[]{shooterLeft, shooterRight}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            /* PIDF coefficients remain untuned placeholders. */
            motor.setVelocityPIDFCoefficients(RobotTuning_RED.SHOOTER_P,
                    RobotTuning_RED.SHOOTER_I, RobotTuning_RED.SHOOTER_D,
                    RobotTuning_RED.SHOOTER_F);
        }
    }
    void initTurret() {
        turret = map.get(Servo.class, "turret");
        turret.setPosition(RobotTuning_RED.TURRET_CENTER);
    }
    void stopInitialized() {
        for (DcMotor m : new DcMotor[]{lf, rf, lb, rb, intake, transfer}) {
            if (m != null) m.setPower(0);
        }
        if (shooterLeft != null) shooterLeft.setVelocity(0);
        if (shooterRight != null) shooterRight.setVelocity(0);
    }
}

/** Each subsystem owns only its functions, not the driver loop. */
final class DrivePart_RED {
    private final RobotParts_RED h;
    DrivePart_RED(RobotParts_RED h) { this.h = h; h.initDrive(); }
    DcMotor[] requirements() { return new DcMotor[]{h.lf, h.rf, h.lb, h.rb}; }
    void update(Gamepad g) {
        double y = -g.left_stick_y, x = g.left_stick_x, rx = g.right_stick_x;
        double denom = Math.max(1, Math.abs(y) + Math.abs(x) + Math.abs(rx));
        double speed = g.left_bumper ? RobotTuning_RED.SLOW_SPEED : RobotTuning_RED.DRIVE_SPEED;
        h.lf.setPower((y + x + rx) / denom * speed);
        h.rf.setPower((y - x - rx) / denom * speed);
        h.lb.setPower((y - x + rx) / denom * speed);
        h.rb.setPower((y + x - rx) / denom * speed);
    }
}
final class IntakePart_RED {
    private final RobotParts_RED h;
    IntakePart_RED(RobotParts_RED h) { this.h = h; h.initIntake(); }
    DcMotor requirement() { return h.intake; }
    void update(Gamepad g) {
        h.intake.setPower(g.right_bumper ? RobotTuning_RED.INTAKE_POWER
                : g.a ? -RobotTuning_RED.INTAKE_POWER : 0);
    }
}
final class TransferPart_RED {
    private final RobotParts_RED h;
    TransferPart_RED(RobotParts_RED h) { this.h = h; h.initTransfer(); }
    DcMotor requirement() { return h.transfer; }
    void update(Gamepad d1, Gamepad d2) {
        h.transfer.setPower(d1.right_bumper || d2.b ? RobotTuning_RED.TRANSFER_POWER
                : d1.a || d2.x ? -RobotTuning_RED.TRANSFER_POWER : 0);
    }
}
final class ShooterPart_RED {
    private final RobotParts_RED h;
    private boolean lastA, enabled;
    private double target = RobotTuning_RED.SHOOTER_LOW;
    ShooterPart_RED(RobotParts_RED h) { this.h = h; h.initShooter(); }
    DcMotorEx[] requirements() { return new DcMotorEx[]{h.shooterLeft, h.shooterRight}; }
    void update(Gamepad g) {
        if (g.a && !lastA) enabled = !enabled;
        lastA = g.a;
        if (g.dpad_up) target = RobotTuning_RED.SHOOTER_HIGH;
        else if (g.dpad_left) target = RobotTuning_RED.SHOOTER_MED;
        else if (g.dpad_down) target = RobotTuning_RED.SHOOTER_LOW;
        h.shooterLeft.setVelocity(enabled ? target : 0);
        h.shooterRight.setVelocity(enabled ? target : 0);
    }
    boolean isEnabled() { return enabled; }
    double target() { return target; }
    double actual() { return (actualLeft() + actualRight()) / 2.0; }
    double actualLeft() { return h.shooterLeft.getVelocity(); }
    double actualRight() { return h.shooterRight.getVelocity(); }
}

/** Pure math. RED mirror: 116 - robotX. BLUE mirror: 172 - robotX. */
final class AimGeometry_RED {
    private AimGeometry_RED() {}
    static final class Result {
        final double targetX, targetY, distance, fieldAngleDeg, turretAngleDeg;
        Result(double x, double y, double d, double f, double t) {
            targetX = x; targetY = y; distance = d;
            fieldAngleDeg = f; turretAngleDeg = t;
        }
    }
    static Result calculate(double robotX, double robotY, double headingRad, double referenceX) {
        double targetX = 2.0 * referenceX - robotX;
        double turretX = robotX + Math.cos(headingRad) * RobotTuning_RED.TURRET_OFFSET_X
                - Math.sin(headingRad) * RobotTuning_RED.TURRET_OFFSET_Y;
        double turretY = robotY + Math.sin(headingRad) * RobotTuning_RED.TURRET_OFFSET_X
                + Math.cos(headingRad) * RobotTuning_RED.TURRET_OFFSET_Y;
        double dx = targetX - turretX, dy = RobotTuning_RED.TARGET_Y - turretY;
        double fieldAngle = Math.atan2(dy, dx);
        double relative = fieldAngle - headingRad
                - Math.toRadians(RobotTuning_RED.TURRET_MOUNT_ZERO_DEG);
        relative = Math.atan2(Math.sin(relative), Math.cos(relative));
        return new Result(targetX, RobotTuning_RED.TARGET_Y, Math.hypot(dx, dy),
                Math.toDegrees(fieldAngle), Math.toDegrees(relative));
    }
    static double clamp(double v, double min, double max) { return Math.max(min, Math.min(max, v)); }
}
final class TurretPart_RED {
    private final RobotParts_RED h;
    private double position = RobotTuning_RED.TURRET_CENTER;
    private boolean limited;
    TurretPart_RED(RobotParts_RED h) { this.h = h; h.initTurret(); }
    Servo requirement() { return h.turret; }
    void manual(Gamepad g) {
        if (Math.abs(g.right_stick_x) > .05)
            position += g.right_stick_x * RobotTuning_RED.TURRET_SPEED;
        set(position);
        limited = false;
    }
    void aim(Pose pose, double referenceX) {
        AimGeometry_RED.Result result = AimGeometry_RED.calculate(
                pose.x(), pose.y(), pose.heading(), referenceX);
        double desired = result.turretAngleDeg;
        double limitedAngle = AimGeometry_RED.clamp(desired,
                -RobotTuning_RED.TURRET_MAX_ANGLE_DEG, RobotTuning_RED.TURRET_MAX_ANGLE_DEG);
        double request = RobotTuning_RED.TURRET_CENTER + RobotTuning_RED.TURRET_DIRECTION
                * limitedAngle / RobotTuning_RED.TURRET_DEGREES_PER_SERVO_UNIT;
        limited = Math.abs(desired - limitedAngle) > .01
                || request < RobotTuning_RED.TURRET_MIN || request > RobotTuning_RED.TURRET_MAX;
        set(request);
    }
    void center() { set(RobotTuning_RED.TURRET_CENTER); limited = false; }
    boolean isLimited() { return limited; }
    private void set(double p) {
        position = AimGeometry_RED.clamp(p, RobotTuning_RED.TURRET_MIN, RobotTuning_RED.TURRET_MAX);
        h.turret.setPosition(position);
    }
}

/** Command factories: Ivy wraps the subsystem functions. */
final class BiobuzzIvy_RED {
    private BiobuzzIvy_RED() {}
    static Command drive(DrivePart_RED s, Gamepad g) {
        return infinite(() -> s.update(g)).requiring(s.requirements());
    }
    static Command intake(IntakePart_RED s, Gamepad g) {
        return infinite(() -> s.update(g)).requiring(s.requirement());
    }
    static Command transfer(TransferPart_RED s, Gamepad g1, Gamepad g2) {
        return infinite(() -> s.update(g1, g2)).requiring(s.requirement());
    }
    static Command shooter(ShooterPart_RED s, Gamepad g) {
        return infinite(() -> s.update(g)).requiring(s.requirements());
    }
    static Command manualTurret(TurretPart_RED s, Gamepad g) {
        return infinite(() -> s.manual(g)).requiring(s.requirement());
    }
    static Command autoTurret(TurretPart_RED s, Follower follower, double referenceX) {
        return infinite(() -> s.aim(follower.pose(), referenceX)).requiring(s.requirement());
    }
}

/** Driver 2: L3 toggles auto, R3 centers, moving right stick cancels auto. */
final class TurretSwitch_RED {
    private final TurretPart_RED turret;
    private final Command manual, auto;
    private boolean autoEnabled, lastL3, lastR3;
    TurretSwitch_RED(TurretPart_RED turret, Command manual, Command auto) {
        this.turret = turret; this.manual = manual; this.auto = auto;
    }
    void update(Gamepad g) {
        boolean l3 = g.left_stick_button, r3 = g.right_stick_button;
        if (r3 && !lastR3) {
            disable(); turret.center();
        } else if (Math.abs(g.right_stick_x) > .05) {
            if (autoEnabled) disable();
        } else if (l3 && !lastL3) {
            if (autoEnabled) disable(); else enable();
        }
        lastL3 = l3; lastR3 = r3;
        if (autoEnabled) {
            if (!Scheduler.isScheduled(auto)) schedule(auto);
        } else if (!Scheduler.isScheduled(manual)) schedule(manual);
    }
    private void enable() { Scheduler.cancel(manual); autoEnabled = true; schedule(auto); }
    private void disable() {
        Scheduler.cancel(auto); autoEnabled = false;
        if (!Scheduler.isScheduled(manual)) schedule(manual);
    }
    boolean isAuto() { return autoEnabled; }
}
