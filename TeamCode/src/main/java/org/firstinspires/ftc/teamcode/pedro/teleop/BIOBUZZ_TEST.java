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

/** Standalone BIOBUZZ TEST OpMode; helper classes are file-local by unique name. */
/** Select a single subsystem with D-pad during INIT, then START. */
@TeleOp(name = "BIOBUZZ TEST")
public class BIOBUZZ_TEST extends LinearOpMode {
    private static final String[] MODES = {
            "AIM MATH (NO HARDWARE)", "DRIVE", "INTAKE", "TRANSFER", "SHOOTER", "TURRET"
    };
    @Override public void runOpMode() {
        int selected = 0;
        boolean lastUp = false, lastDown = false;
        while (opModeInInit() && !isStopRequested()) {
            if (gamepad1.dpad_up && !lastUp) selected = (selected + MODES.length - 1) % MODES.length;
            if (gamepad1.dpad_down && !lastDown) selected = (selected + 1) % MODES.length;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            telemetry.addLine("D-pad UP/DOWN: choose test; START: run");
            telemetry.addData("Selected", MODES[selected]);
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;
        Scheduler.reset();
        RobotParts_TEST hw = new RobotParts_TEST(hardwareMap);
        try {
            if (selected == 0) {
                runMathTest();
                return;
            }
            Command command;
            switch (selected) {
                case 1: command = BiobuzzIvy_TEST.drive(new DrivePart_TEST(hw), gamepad1); break;
                case 2: command = BiobuzzIvy_TEST.intake(new IntakePart_TEST(hw), gamepad1); break;
                case 3: command = BiobuzzIvy_TEST.transfer(new TransferPart_TEST(hw), gamepad1, gamepad2); break;
                case 4: command = BiobuzzIvy_TEST.shooter(new ShooterPart_TEST(hw), gamepad2); break;
                default: command = BiobuzzIvy_TEST.manualTurret(new TurretPart_TEST(hw), gamepad2); break;
            }
            schedule(command);
            while (opModeIsActive()) {
                Scheduler.execute();
                telemetry.addData("TEST", MODES[selected]);
                telemetry.addLine("Only selected mechanism is initialized");
                telemetry.update();
            }
        } finally {
            Scheduler.reset();
            hw.stopInitialized();
        }
    }
    private void runMathTest() {
        double x = 24, y = 24, heading = 0;
        while (opModeIsActive()) {
            x = AimGeometry_TEST.clamp(x + gamepad1.left_stick_x * .3, 0, 144);
            y = AimGeometry_TEST.clamp(y - gamepad1.left_stick_y * .3, 0, 144);
            heading += gamepad1.right_stick_x * .01;
            AimGeometry_TEST.Result red = AimGeometry_TEST.calculate(x, y, heading, 58);
            AimGeometry_TEST.Result blue = AimGeometry_TEST.calculate(x, y, heading, 86);
            telemetry.addLine("Left stick: move simulated X/Y; right X: heading");
            telemetry.addData("Robot", "X %.1f Y %.1f H %.1f", x, y, Math.toDegrees(heading));
            telemetry.addData("RED target", "(%.1f, %.1f) angle %.1f", red.targetX, red.targetY, red.turretAngleDeg);
            telemetry.addData("BLUE target", "(%.1f, %.1f) angle %.1f", blue.targetX, blue.targetY, blue.turretAngleDeg);
            telemetry.update();
            idle();
        }
    }
}


/** All numbers that will need robot-side calibration. */
final class RobotTuning_TEST {
    private RobotTuning_TEST() {}
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
final class RobotParts_TEST {
    final HardwareMap map;
    DcMotor lf, rf, lb, rb, intake, transfer;
    DcMotorEx shooterLeft, shooterRight;
    Servo turret;
    RobotParts_TEST(HardwareMap map) { this.map = map; }
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
            motor.setVelocityPIDFCoefficients(RobotTuning_TEST.SHOOTER_P,
                    RobotTuning_TEST.SHOOTER_I, RobotTuning_TEST.SHOOTER_D,
                    RobotTuning_TEST.SHOOTER_F);
        }
    }
    void initTurret() {
        turret = map.get(Servo.class, "turret");
        turret.setPosition(RobotTuning_TEST.TURRET_CENTER);
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
final class DrivePart_TEST {
    private final RobotParts_TEST h;
    DrivePart_TEST(RobotParts_TEST h) { this.h = h; h.initDrive(); }
    DcMotor[] requirements() { return new DcMotor[]{h.lf, h.rf, h.lb, h.rb}; }
    void update(Gamepad g) {
        double y = -g.left_stick_y, x = g.left_stick_x, rx = g.right_stick_x;
        double denom = Math.max(1, Math.abs(y) + Math.abs(x) + Math.abs(rx));
        double speed = g.left_bumper ? RobotTuning_TEST.SLOW_SPEED : RobotTuning_TEST.DRIVE_SPEED;
        h.lf.setPower((y + x + rx) / denom * speed);
        h.rf.setPower((y - x - rx) / denom * speed);
        h.lb.setPower((y - x + rx) / denom * speed);
        h.rb.setPower((y + x - rx) / denom * speed);
    }
}
final class IntakePart_TEST {
    private final RobotParts_TEST h;
    IntakePart_TEST(RobotParts_TEST h) { this.h = h; h.initIntake(); }
    DcMotor requirement() { return h.intake; }
    void update(Gamepad g) {
        h.intake.setPower(g.right_bumper ? RobotTuning_TEST.INTAKE_POWER
                : g.a ? -RobotTuning_TEST.INTAKE_POWER : 0);
    }
}
final class TransferPart_TEST {
    private final RobotParts_TEST h;
    TransferPart_TEST(RobotParts_TEST h) { this.h = h; h.initTransfer(); }
    DcMotor requirement() { return h.transfer; }
    void update(Gamepad d1, Gamepad d2) {
        h.transfer.setPower(d1.right_bumper || d2.b ? RobotTuning_TEST.TRANSFER_POWER
                : d1.a || d2.x ? -RobotTuning_TEST.TRANSFER_POWER : 0);
    }
}
final class ShooterPart_TEST {
    private final RobotParts_TEST h;
    private boolean lastA, enabled;
    private double target = RobotTuning_TEST.SHOOTER_LOW;
    ShooterPart_TEST(RobotParts_TEST h) { this.h = h; h.initShooter(); }
    DcMotorEx[] requirements() { return new DcMotorEx[]{h.shooterLeft, h.shooterRight}; }
    void update(Gamepad g) {
        if (g.a && !lastA) enabled = !enabled;
        lastA = g.a;
        if (g.dpad_up) target = RobotTuning_TEST.SHOOTER_HIGH;
        else if (g.dpad_left) target = RobotTuning_TEST.SHOOTER_MED;
        else if (g.dpad_down) target = RobotTuning_TEST.SHOOTER_LOW;
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
final class AimGeometry_TEST {
    private AimGeometry_TEST() {}
    static final class Result {
        final double targetX, targetY, distance, fieldAngleDeg, turretAngleDeg;
        Result(double x, double y, double d, double f, double t) {
            targetX = x; targetY = y; distance = d;
            fieldAngleDeg = f; turretAngleDeg = t;
        }
    }
    static Result calculate(double robotX, double robotY, double headingRad, double referenceX) {
        double targetX = 2.0 * referenceX - robotX;
        double turretX = robotX + Math.cos(headingRad) * RobotTuning_TEST.TURRET_OFFSET_X
                - Math.sin(headingRad) * RobotTuning_TEST.TURRET_OFFSET_Y;
        double turretY = robotY + Math.sin(headingRad) * RobotTuning_TEST.TURRET_OFFSET_X
                + Math.cos(headingRad) * RobotTuning_TEST.TURRET_OFFSET_Y;
        double dx = targetX - turretX, dy = RobotTuning_TEST.TARGET_Y - turretY;
        double fieldAngle = Math.atan2(dy, dx);
        double relative = fieldAngle - headingRad
                - Math.toRadians(RobotTuning_TEST.TURRET_MOUNT_ZERO_DEG);
        relative = Math.atan2(Math.sin(relative), Math.cos(relative));
        return new Result(targetX, RobotTuning_TEST.TARGET_Y, Math.hypot(dx, dy),
                Math.toDegrees(fieldAngle), Math.toDegrees(relative));
    }
    static double clamp(double v, double min, double max) { return Math.max(min, Math.min(max, v)); }
}
final class TurretPart_TEST {
    private final RobotParts_TEST h;
    private double position = RobotTuning_TEST.TURRET_CENTER;
    private boolean limited;
    TurretPart_TEST(RobotParts_TEST h) { this.h = h; h.initTurret(); }
    Servo requirement() { return h.turret; }
    void manual(Gamepad g) {
        if (Math.abs(g.right_stick_x) > .05)
            position += g.right_stick_x * RobotTuning_TEST.TURRET_SPEED;
        set(position);
        limited = false;
    }
    void aim(Pose pose, double referenceX) {
        AimGeometry_TEST.Result result = AimGeometry_TEST.calculate(
                pose.x(), pose.y(), pose.heading(), referenceX);
        double desired = result.turretAngleDeg;
        double limitedAngle = AimGeometry_TEST.clamp(desired,
                -RobotTuning_TEST.TURRET_MAX_ANGLE_DEG, RobotTuning_TEST.TURRET_MAX_ANGLE_DEG);
        double request = RobotTuning_TEST.TURRET_CENTER + RobotTuning_TEST.TURRET_DIRECTION
                * limitedAngle / RobotTuning_TEST.TURRET_DEGREES_PER_SERVO_UNIT;
        limited = Math.abs(desired - limitedAngle) > .01
                || request < RobotTuning_TEST.TURRET_MIN || request > RobotTuning_TEST.TURRET_MAX;
        set(request);
    }
    void center() { set(RobotTuning_TEST.TURRET_CENTER); limited = false; }
    boolean isLimited() { return limited; }
    private void set(double p) {
        position = AimGeometry_TEST.clamp(p, RobotTuning_TEST.TURRET_MIN, RobotTuning_TEST.TURRET_MAX);
        h.turret.setPosition(position);
    }
}

/** Command factories: Ivy wraps the subsystem functions. */
final class BiobuzzIvy_TEST {
    private BiobuzzIvy_TEST() {}
    static Command drive(DrivePart_TEST s, Gamepad g) {
        return infinite(() -> s.update(g)).requiring(s.requirements());
    }
    static Command intake(IntakePart_TEST s, Gamepad g) {
        return infinite(() -> s.update(g)).requiring(s.requirement());
    }
    static Command transfer(TransferPart_TEST s, Gamepad g1, Gamepad g2) {
        return infinite(() -> s.update(g1, g2)).requiring(s.requirement());
    }
    static Command shooter(ShooterPart_TEST s, Gamepad g) {
        return infinite(() -> s.update(g)).requiring(s.requirements());
    }
    static Command manualTurret(TurretPart_TEST s, Gamepad g) {
        return infinite(() -> s.manual(g)).requiring(s.requirement());
    }
    static Command autoTurret(TurretPart_TEST s, Follower follower, double referenceX) {
        return infinite(() -> s.aim(follower.pose(), referenceX)).requiring(s.requirement());
    }
}

/** Driver 2: L3 toggles auto, R3 centers, moving right stick cancels auto. */
final class TurretSwitch_TEST {
    private final TurretPart_TEST turret;
    private final Command manual, auto;
    private boolean autoEnabled, lastL3, lastR3;
    TurretSwitch_TEST(TurretPart_TEST turret, Command manual, Command auto) {
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
