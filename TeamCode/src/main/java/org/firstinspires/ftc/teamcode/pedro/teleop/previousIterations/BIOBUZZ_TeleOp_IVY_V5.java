
package org.firstinspires.ftc.teamcode.pedro.teleop.previousIterations;

/*
 * BIOBUZZ TELEOP V5 - PEDRO PATHING + IVY
 *
 * PURPOSE:
 * This program controls our BIOBUZZ robot during TeleOp.
 *
 * We use:
 * 1. Pedro Pathing to estimate the robot's field position.
 * 2. Ivy to organize and schedule robot commands.
 * 3. Gamepads to control the drivetrain and mechanisms.
 *
 * IMPORTANT IVY CONCEPT:
 *
 * A COMMAND is a task that controls part of the robot.
 *
 * Examples:
 * - Drive Command: reads joysticks and moves the drivetrain.
 * - Shooter Command: controls shooter velocity.
 * - Manual Turret Command: moves the turret with the joystick.
 * - Auto Aim Command: automatically points toward the HIVE.
 *
 * Ivy helps prevent two commands from controlling the same
 * mechanism at the same time.
 *
 * DRIVER CONTROLS:
 *
 * DRIVER 1:
 * Left Stick       = Drive / Strafe
 * Right Stick X    = Rotate
 * Left Bumper      = Slow Mode
 * Right Bumper     = Intake + Transfer
 * A                = Reverse / Unjam
 *
 * DRIVER 2:
 * A                = Toggle Shooter
 * D-Pad Up         = High Shooter Velocity
 * D-Pad Left       = Medium Shooter Velocity
 * D-Pad Down       = Low Shooter Velocity
 * B                = Transfer Forward
 * X                = Transfer Reverse
 * Right Stick X    = Manual Turret
 * L3               = Toggle HIVE Auto Aim
 * R3               = Center Turret
 */


/*
 * FTC SDK imports
 */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * Pedro Pathing imports
 */
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.api.PoseFactory;

/*
 * Ivy command scheduler imports
 */
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

/*
 * Allows us to write schedule(command)
 */
import static com.pedropathing.ivy.Scheduler.schedule;

/*
 * Creates a command that runs continuously until canceled
 */
import static com.pedropathing.ivy.commands.Commands.infinite;

/*
 * Our team's Pedro configuration
 */
import org.firstinspires.ftc.teamcode.pedro.misc.Constants;


@TeleOp(name = "BIOBUZZ IVY V5")
public class BIOBUZZ_TeleOp_IVY_V5 extends LinearOpMode {


    /*
     * SECTION 1: ALLIANCE SELECTION
     *
     * This allows us to use ONE TeleOp for both alliances.
     *
     * true  = BLUE alliance
     * false = RED alliance
     *
     * The auto-aim system chooses the correct HIVE coordinates
     * based on this variable.
     *
     * IMPORTANT:
     * Change this before the match if using this version.
     */

    private static final boolean BLUE_ALLIANCE = true;


    /*
     * SECTION 2: DRIVE SETTINGS
     *
     * Maximum driving power (100%)
     */
    private static final double DRIVE_SPEED = 1.0;

    /*
     * Reduced power for precise movements (40%)
     */
    private static final double SLOW_SPEED = 0.40;


    /*
     * SECTION 3: INTAKE AND TRANSFER SETTINGS
     *
     * Positive power runs mechanisms forward.
     * If a mechanism moves backward, its direction may
     * need to be reversed in hardware setup or code.
     */

    private static final double INTAKE_POWER = 1.0;

    private static final double TRANSFER_POWER = 1.0;


    /*
     * SECTION 4: SHOOTER SETTINGS
     *
     * Shooter velocity is measured in encoder ticks/second.
     *
     * These are starting values, NOT finalized shot speeds.
     *
     * The driver can select one of three target velocities.
     */

    private static final double SHOOTER_LOW = 1200;

    private static final double SHOOTER_MED = 1500;

    private static final double SHOOTER_HIGH = 1800;

    /*
        // --------------------------------------------------------
        // SHOOTER PIDF
        // --------------------------------------------------------
        //
        // PIDF helps the motor reach and maintain target velocity.
        //
        // P = Corrects current velocity error.
        // I = Corrects accumulated error over time.
        // D = Reacts to how quickly the error changes.
        // F = Feedforward estimate of required motor output.
        //
        // TODO:
        // Replace these placeholders with values from testing.
        //
        // All-zero values are NOT a tuned shooter controller.
        // --------------------------------------------------------
    */
    private static final double P = 0.0;
    private static final double I = 0.0;
    private static final double D = 0.0;
    private static final double F = 0.0;


    /*
     * SECTION 5: TURRET SETTINGS
     *
     * Servo position representing straight ahead
     */
    private static final double TURRET_CENTER = 0.50;

    /*
     * Software limits to prevent excessive rotation.
     *
     * Servo position usually ranges from 0.0 to 1.0.
     *
     * We are intentionally restricting the range during
     * initial testing.
     */
    private static final double TURRET_MIN = 0.20;
    private static final double TURRET_MAX = 0.80;

    /*
     * Amount the servo position changes per control loop
     * while the right joystick is moved.
     *
     * NOTE:
     * Manual turret speed currently depends on loop frequency.
     * We can make it time-based in a later iteration.
     */
    private static final double TURRET_SPEED = 0.01;

    /*
     * Changes which direction the turret rotates.
     *
     * 1.0 = Normal direction
     * -1.0 = Reverse direction
     *
     * Change only after verifying physical movement.
     */
    private static final double TURRET_DIRECTION = 1.0;


    /*
     * SECTION 6: HIVE FIELD COORDINATES
     *
     * Pedro uses field coordinates to estimate where the
     * robot is located.
     *
     * We also need coordinates for the scoring target.
     *
     * The turret uses the robot's position and the HIVE
     * position to calculate which direction to point.
     *
     * IMPORTANT:
     * These are our current coordinate assumptions.
     * Verify against the actual Pedro field coordinate system.
     */

    private static final double BLUE_HIVE_X = 72.0;
    private static final double BLUE_HIVE_Y = 59.25;

    private static final double RED_HIVE_X = 72.0;
    private static final double RED_HIVE_Y = 84.75;


    /*
     * SECTION 7: ROBOT HARDWARE
     *
     * These variables represent physical robot devices.
     *
     * The names used in hardwareMap must match the names
     * configured on the Robot Controller.
     *
     * Mecanum drivetrain motors
     */
    private DcMotor LF;
    private DcMotor RF;
    private DcMotor LB;
    private DcMotor RB;

    /*
     * Game-piece handling mechanisms
     */
    private DcMotor intake;
    private DcMotor transfer;

    /*
     * DcMotorEx supports encoder velocity control
     */
    private DcMotorEx shooter;

    /*
     * Axon positional turret servo
     */
    private Servo turret;


    /*
     * SECTION 8: PEDRO PATHING
     *
     * Follower:
     * Maintains Pedro's robot pose estimate.
     *
     * Pose:
     * Represents robot X, Y, and heading.
     *
     * PoseFactory.degrees():
     * Allows us to define starting headings in degrees.
     */

    private Follower follower;

    private final PoseFactory poseFactory =
            PoseFactory.degrees();


    /*
     * TODO:
     * Replace with the actual TeleOp starting position.
     *
     * Incorrect starting coordinates will cause auto-aim
     * to calculate incorrect turret angles.
     */
    private final Pose startPose =
            poseFactory.of(24, 24, 0);


    /*
     * SECTION 9: IVY COMMAND VARIABLES
     *
     * We create one continuously running command for each
     * major mechanism.
     *
     * The turret is special:
     * It has TWO different commands controlling the SAME servo.
     *
     * Ivy's command requirements help us coordinate ownership.
     */

    private Command driveCommand;

    private Command intakeCommand;

    private Command transferCommand;

    private Command shooterCommand;

    /*
     * Turret controlled using right joystick
     */
    private Command manualTurretCommand;

    /*
     * Turret automatically tracks the HIVE
     */
    private Command autoAimCommand;


    /*
     * SECTION 10: ROBOT STATE VARIABLES
     *
     * State variables remember information between loops.
     *
     * Example:
     * shooterOn remembers whether the shooter was toggled on.
     */

    private boolean shooterOn = false;

    private double shooterTarget = SHOOTER_LOW;


    /*
     * Stores the previous A-button state.
     *
     * Used to detect a NEW press instead of repeatedly
     * toggling while the button is held.
     */
    private boolean lastShooterA = false;


    /*
     * Whether automatic turret tracking is active
     */
    private boolean autoAimEnabled = false;


    /*
     * Previous joystick-button states for edge detection
     */
    private boolean lastL3 = false;
    private boolean lastR3 = false;


    /*
     * Current commanded servo position
     */
    private double turretPosition = TURRET_CENTER;


    /*
     * Calculated robot-relative angle toward the HIVE
     */
    private double turretAngle = 0.0;


    /*
     * Direction to HIVE in field coordinates
     */
    private double angleToHive = 0.0;


    /*
     * Distance between robot and HIVE
     */
    private double distanceToHive = 0.0;


    /*
     * True if turret cannot reach the requested direction
     * because of mechanical/software limits
     */
    private boolean aimLimited = false;


    /*
     * SECTION 11: MAIN OPMODE
     *
     * runOpMode() executes when the driver selects this OpMode.
     *
     * Main steps:
     * 1. Reset Ivy
     * 2. Initialize hardware
     * 3. Initialize Pedro
     * 4. Create commands
     * 5. Wait for START
     * 6. Schedule commands
     * 7. Continuously update Pedro and Ivy
     */

    @Override
    public void runOpMode() {

        /*
         * Clear previously scheduled commands
         */
        Scheduler.reset();


        /*
         * HARDWARE INITIALIZATION
         */

        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");

        intake = hardwareMap.get(DcMotor.class, "intake");

        transfer = hardwareMap.get(DcMotor.class, "transfer");

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        turret = hardwareMap.get(Servo.class, "turret");


        /*
         * MOTOR DIRECTIONS
         *
         * Motors mounted on opposite sides of a drivetrain
         * often need opposite software directions.
         *
         * These directions are based on our current setup.
         */

        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);

        RF.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.FORWARD);


        /*
         * ZERO POWER BEHAVIOR
         *
         * BRAKE:
         * Resists movement when motor power is zero.
         *
         * FLOAT:
         * Allows the motor to coast when power is zero.
         */

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


        /*
         * SHOOTER INITIALIZATION
         *
         * Reset the encoder count and enable velocity control.
         */

        shooter.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        shooter.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );


        /*
         * Apply PIDF constants
         */
        shooter.setVelocityPIDFCoefficients(P, I, D, F);


        /*
         * TURRET INITIALIZATION
         */

        turretPosition = TURRET_CENTER;

        turret.setPosition(turretPosition);


        /*
         * PEDRO INITIALIZATION
         *
         * Create the follower using team configuration
         */
        follower = Constants.create(hardwareMap);

        /*
         * Tell Pedro where the robot starts
         */
        follower.setPose(startPose);


        /*
         * CREATE IVY COMMANDS
         */

        createCommands();


        telemetry.addLine("BIOBUZZ IVY V5 READY");

        telemetry.addData(
                "Alliance",
                BLUE_ALLIANCE ? "BLUE" : "RED"
        );

        telemetry.addLine("L3 = HIVE Auto Aim");
        telemetry.addLine("R3 = Center Turret");

        telemetry.update();


        /*
         * Wait for the driver to press START
         */
        waitForStart();

        if (isStopRequested()) {
            Scheduler.reset();
            return;
        }


        /*
         * SCHEDULE DEFAULT MANUAL COMMANDS
         *
         * These commands continuously read the gamepads.
         *
         * Instead of calling each mechanism method ourselves,
         * we let Ivy run the commands.
         */

        schedule(
                driveCommand,
                intakeCommand,
                transferCommand,
                shooterCommand,
                manualTurretCommand
        );


        /*
         * MAIN TELEOP LOOP
         */

        try {

            while (opModeIsActive()) {

                /*
                 * STEP 1:
                 * Update Pedro's estimate of robot position.
                 */
                follower.update();


                /*
                 * STEP 2:
                 * Detect turret button presses and decide
                 * whether manual or auto-aim should run.
                 */
                handleTurretButtons();


                /*
                 * STEP 3:
                 * Let Ivy execute the active commands.
                 */
                Scheduler.execute();


                /*
                 * STEP 4:
                 * Display robot information to drivers.
                 */
                updateTelemetry();
            }

        } finally {

            /*
             * STOP ROBOT
             *
             * This block executes when TeleOp exits,
             * including if an exception occurs.
             */

            Scheduler.reset();

            LF.setPower(0);
            RF.setPower(0);
            LB.setPower(0);
            RB.setPower(0);

            intake.setPower(0);
            transfer.setPower(0);

            shooter.setPower(0);
        }
    }


    /*
     * SECTION 12: CREATE IVY COMMANDS
     *
     * infinite():
     * Creates a command that continuously executes a method.
     *
     * requiring():
     * Declares which hardware devices the command controls.
     *
     * Why requirements matter:
     *
     * Both manualTurretCommand and autoAimCommand require
     * the same turret servo.
     *
     * This helps Ivy prevent both commands from controlling
     * the turret simultaneously.
     *
     * These commands are CREATED here, not executed yet.
     */

    private void createCommands() {

        driveCommand = infinite(this::driveControl)
                .requiring(LF, RF, LB, RB);


        intakeCommand = infinite(this::intakeControl)
                .requiring(intake);


        transferCommand = infinite(this::transferControl)
                .requiring(transfer);


        shooterCommand = infinite(this::shooterControl)
                .requiring(shooter);


        /*
         * Normal joystick-based turret control
         */
        manualTurretCommand = infinite(this::manualTurret)
                .requiring(turret);


        /*
         * Automatic turret tracking using Pedro localization
         */
        autoAimCommand = infinite(this::aimAtHive)
                .requiring(turret);
    }


    /*
     * SECTION 13: DRIVETRAIN COMMAND
     *
     * Controlled by Driver 1.
     *
     * LEFT STICK:
     * Y controls forward/backward.
     * X controls strafing.
     *
     * RIGHT STICK X:
     * Controls rotation.
     *
     * LEFT BUMPER:
     * Reduces speed for precise positioning.
     *
     * Mecanum wheel power formulas combine these movements.
     */

    private void driveControl() {

        /*
         * Gamepad Y is inverted so pushing forward is positive
         */
        double y = -gamepad1.left_stick_y;

        double x = gamepad1.left_stick_x;

        double rx = gamepad1.right_stick_x;


        /*
         * Prevent motor powers from exceeding their range.
         *
         * If the combined joystick inputs are greater than 1,
         * divide all wheel powers by the same amount.
         *
         * This preserves the intended movement proportions.
         */
        double denominator = Math.max(
                Math.abs(y) + Math.abs(x) + Math.abs(rx),
                1.0
        );


        /*
         * Select driving speed
         */
        double speed = gamepad1.left_bumper
                ? SLOW_SPEED
                : DRIVE_SPEED;


        /*
         * Calculate individual mecanum wheel powers
         */
        LF.setPower(
                (y + x + rx) / denominator * speed
        );

        RF.setPower(
                (y - x - rx) / denominator * speed
        );

        LB.setPower(
                (y - x + rx) / denominator * speed
        );

        RB.setPower(
                (y + x - rx) / denominator * speed
        );
    }


    /*
     * SECTION 14: INTAKE COMMAND
     *
     * Driver 1 Right Bumper:
     * Runs intake forward to collect game pieces.
     *
     * Driver 1 A:
     * Reverses intake to clear jams.
     *
     * Otherwise:
     * Stops the intake.
     */

    private void intakeControl() {

        if (gamepad1.right_bumper) {

            intake.setPower(INTAKE_POWER);

        } else if (gamepad1.a) {

            intake.setPower(-INTAKE_POWER);

        } else {

            intake.setPower(0);
        }
    }


    /*
     * SECTION 15: TRANSFER COMMAND
     *
     * Transfer moves game pieces toward the shooter.
     *
     * Forward:
     * Driver 1 Right Bumper OR Driver 2 B
     *
     * Reverse:
     * Driver 1 A OR Driver 2 X
     *
     * Forward currently takes priority if both are pressed.
     */

    private void transferControl() {

        if (gamepad1.right_bumper || gamepad2.b) {

            transfer.setPower(TRANSFER_POWER);

        } else if (gamepad1.a || gamepad2.x) {

            transfer.setPower(-TRANSFER_POWER);

        } else {

            transfer.setPower(0);
        }
    }


    /*
     * SECTION 16: SHOOTER COMMAND
     *
     * Driver 2 A:
     * Toggles shooter ON/OFF.
     *
     * Driver 2 D-Pad:
     * UP   = High velocity
     * LEFT = Medium velocity
     * DOWN = Low velocity
     *
     * The shooter uses encoder velocity control rather than
     * simply setting a fixed motor power.
     */

    private void shooterControl() {

        /*
         * SHOOTER TOGGLE
         *
         * The condition checks:
         * A is pressed NOW and was NOT pressed last loop.
         *
         * This prevents repeatedly toggling while A is held.
         */

        if (gamepad2.a && !lastShooterA) {

            shooterOn = !shooterOn;
        }

        lastShooterA = gamepad2.a;


        /*
         * SELECT VELOCITY
         */

        if (gamepad2.dpad_up) {

            shooterTarget = SHOOTER_HIGH;

        } else if (gamepad2.dpad_left) {

            shooterTarget = SHOOTER_MED;

        } else if (gamepad2.dpad_down) {

            shooterTarget = SHOOTER_LOW;
        }


        /*
         * APPLY VELOCITY
         */

        if (shooterOn) {

            shooter.setVelocity(shooterTarget);

        } else {

            shooter.setVelocity(0);
        }
    }


    /*
     * SECTION 17: TURRET BUTTON HANDLER
     *
     * This method decides WHICH Ivy turret command runs.
     *
     * It does not continuously calculate servo positions.
     * Instead, it manages command transitions.
     *
     * PRIORITIES:
     *
     * 1. R3 centers the turret.
     * 2. Manual joystick overrides auto-aim.
     * 3. L3 toggles auto-aim.
     *
     * This is important because the driver must always
     * be able to regain manual control.
     */

    private void handleTurretButtons() {

        boolean l3 = gamepad2.left_stick_button;

        boolean r3 = gamepad2.right_stick_button;

        double manualInput = gamepad2.right_stick_x;


        /*
         * PRIORITY 1: CENTER TURRET
         */

        if (r3 && !lastR3) {

            /*
             * Stop auto-aim and return to manual control
             */
            disableAutoAim();

            /*
             * Command the turret to its center position
             */
            turretPosition = TURRET_CENTER;

            turret.setPosition(turretPosition);


            /*
             * PRIORITY 2: MANUAL OVERRIDE
             */

        } else if (Math.abs(manualInput) > 0.05) {

            /*
             * If the driver moves the stick during auto-aim,
             * stop automatic tracking immediately.
             */
            if (autoAimEnabled) {

                disableAutoAim();
            }


            /*
             * PRIORITY 3: AUTO-AIM TOGGLE
             */

        } else if (l3 && !lastL3) {

            if (autoAimEnabled) {

                /*
                 * Already aiming automatically:
                 * turn auto-aim OFF
                 */
                disableAutoAim();

            } else {

                /*
                 * Currently manual:
                 * turn auto-aim ON
                 */
                enableAutoAim();
            }
        }


        /*
         * Save button states for the next loop
         */
        lastL3 = l3;
        lastR3 = r3;


        /*
         * ENSURE CORRECT COMMAND IS SCHEDULED
         *
         * If auto-aim is OFF, manual control should run.
         *
         * If auto-aim is ON, the automatic command should run.
         *
         * isScheduled() prevents unnecessary rescheduling.
         */

        if (!autoAimEnabled) {

            if (!Scheduler.isScheduled(manualTurretCommand)) {

                schedule(manualTurretCommand);
            }

        } else {

            if (!Scheduler.isScheduled(autoAimCommand)) {

                schedule(autoAimCommand);
            }
        }
    }


    /*
     * SECTION 18: ENABLE AUTO AIM
     *
     * Called when Driver 2 presses L3 to activate auto-aim.
     *
     * Step 1: Cancel manual turret control.
     * Step 2: Record that auto-aim is enabled.
     * Step 3: Schedule the automatic tracking command.
     *
     * Both commands require the same turret servo.
     */

    private void enableAutoAim() {

        Scheduler.cancel(manualTurretCommand);

        autoAimEnabled = true;

        schedule(autoAimCommand);
    }


    /*
     * SECTION 19: DISABLE AUTO AIM
     *
     * Called when:
     * - Driver presses L3 again.
     * - Driver moves the right joystick.
     * - Driver presses R3.
     *
     * Ivy cancels automatic tracking and restores manual
     * turret control.
     */

    private void disableAutoAim() {

        Scheduler.cancel(autoAimCommand);

        autoAimEnabled = false;

        aimLimited = false;


        /*
         * Return control to the driver
         */
        if (!Scheduler.isScheduled(manualTurretCommand)) {

            schedule(manualTurretCommand);
        }
    }


    /*
     * SECTION 20: MANUAL TURRET COMMAND
     *
     * This method runs continuously while manualTurretCommand
     * is active.
     *
     * Driver 2 Right Stick X controls turret rotation.
     *
     * Unlike auto-aim, this command does not use Pedro.
     * It simply adjusts the current servo position.
     */

    private void manualTurret() {

        double input = gamepad2.right_stick_x;


        /*
         * Ignore tiny joystick movements (deadzone)
         */
        if (Math.abs(input) > 0.05) {

            turretPosition += input * TURRET_SPEED;
        }


        /*
         * Restrict servo to allowed range
         */
        turretPosition = clampTurret(turretPosition);


        /*
         * Send desired position to the Axon servo
         */
        turret.setPosition(turretPosition);
    }


    /*
     * SECTION 21: AUTOMATIC HIVE AIMING
     *
     * This is the main auto-aim calculation.
     *
     * It runs continuously while autoAimCommand is active.
     *
     * GOAL:
     * Rotate the turret toward the correct alliance HIVE.
     *
     * PROCESS:
     *
     * 1. Read robot position from Pedro.
     * 2. Select the alliance HIVE coordinates.
     * 3. Calculate distance to the HIVE.
     * 4. Calculate field-relative target angle.
     * 5. Subtract robot heading.
     * 6. Normalize the resulting angle.
     * 7. Convert angle into servo position.
     * 8. Apply turret limits.
     *
     * This aims horizontally.
     * It does NOT automatically change shooter velocity.
     */

    private void aimAtHive() {


        /*
         * STEP 1: GET ROBOT POSITION
         *
         * Pedro estimates:
         * X = Horizontal field coordinate
         * Y = Vertical field coordinate
         * Heading = Robot orientation in radians
         */

        Pose pose = follower.pose();


        /*
         * STEP 2: SELECT ALLIANCE HIVE
         */

        double targetX = BLUE_ALLIANCE
                ? BLUE_HIVE_X
                : RED_HIVE_X;

        double targetY = BLUE_ALLIANCE
                ? BLUE_HIVE_Y
                : RED_HIVE_Y;


        /*
         * STEP 3: CALCULATE X AND Y DIFFERENCES
         *
         * dx = Horizontal distance from robot to target.
         * dy = Vertical distance from robot to target.
         *
         * These are signed differences.
         */

        double dx = targetX - pose.x();

        double dy = targetY - pose.y();


        /*
         * STEP 4: CALCULATE DISTANCE
         *
         * Math.hypot(dx, dy) calculates:
         *
         * distance = sqrt(dx^2 + dy^2)
         *
         * We can later use this distance to automatically
         * adjust shooter velocity.
         */

        distanceToHive = Math.hypot(dx, dy);


        /*
         * STEP 5: CALCULATE FIELD ANGLE
         *
         * atan2(dy, dx) calculates the direction from the
         * robot's position toward the HIVE.
         *
         * atan2 returns an angle in radians.
         *
         * This angle is relative to the FIELD, not the robot.
         */

        double fieldAngle = Math.atan2(dy, dx);


        /*
         * STEP 6: ACCOUNT FOR ROBOT HEADING
         *
         * Suppose:
         *
         * HIVE direction on field = 60 degrees
         * Robot heading           = 20 degrees
         *
         * Required turret angle:
         *
         * 60 - 20 = 40 degrees
         *
         * This makes the turret compensate as the robot turns.
         */

        double relativeAngle =
                fieldAngle - pose.heading();


        /*
         * STEP 7: NORMALIZE ANGLE
         *
         * Angles wrap around after a full revolution.
         *
         * For example:
         * 350 degrees is equivalent to -10 degrees.
         *
         * These loops keep the angle between -180 and +180.
         *
         * Math.PI radians = 180 degrees.
         */

        while (relativeAngle > Math.PI) {

            relativeAngle -= 2.0 * Math.PI;
        }

        while (relativeAngle < -Math.PI) {

            relativeAngle += 2.0 * Math.PI;
        }


        /*
         * Save field angle for telemetry
         */
        angleToHive = Math.toDegrees(fieldAngle);


        /*
         * Convert robot-relative angle to degrees
         */
        turretAngle = Math.toDegrees(relativeAngle);


        /*
         * STEP 8: LIMIT REQUESTED TURRET ANGLE
         *
         * We currently assume the turret has 180 degrees
         * of total travel:
         *
         * -90 degrees to +90 degrees.
         *
         * If the target is outside this range, the turret
         * cannot point directly at it.
         */

        double limitedAngle = Math.max(
                -90.0,
                Math.min(90.0, turretAngle)
        );


        /*
         * STEP 9: CONVERT ANGLE TO SERVO POSITION
         *
         * Our INITIAL assumed mapping is:
         *
         * -90 degrees -> servo 0.00
         * 0 degrees -> servo 0.50
         * +90 degrees -> servo 1.00
         *
         * This assumes the turret's actual mechanical travel
         * matches the servo position mapping.
         *
         * We still need to calibrate this on the robot.
         */

        double requestedPosition =
                TURRET_CENTER
                        + TURRET_DIRECTION
                        * (limitedAngle / 180.0);


        /*
         * STEP 10: DETECT AIM LIMITS
         *
         * aimLimited becomes true when:
         *
         * 1. The target angle exceeds +/-90 degrees.
         * OR
         * 2. The servo position exceeds software limits.
         *
         * This is useful because auto-aim being ON does not
         * guarantee the turret can physically reach the HIVE.
         */

        aimLimited =
                Math.abs(turretAngle - limitedAngle) > 0.01
                        || requestedPosition < TURRET_MIN
                        || requestedPosition > TURRET_MAX;


        /*
         * STEP 11: APPLY LIMITS AND MOVE TURRET
         */

        turretPosition = clampTurret(requestedPosition);

        turret.setPosition(turretPosition);
    }


    /*
     * SECTION 22: TURRET CLAMP HELPER
     *
     * A clamp prevents a number from going below a minimum
     * or above a maximum.
     *
     * Example:
     *
     * Allowed range: 0.20 to 0.80
     *
     * Requested 0.95 -> Output 0.80
     * Requested 0.10 -> Output 0.20
     * Requested 0.50 -> Output 0.50
     *
     * This protects against software commands outside the
     * allowed range, but physical clearance must still be
     * verified on the actual robot.
     */

    private double clampTurret(double position) {

        return Math.max(
                TURRET_MIN,
                Math.min(TURRET_MAX, position)
        );
    }


    /*
     * SECTION 23: TELEMETRY
     *
     * Telemetry displays robot information on the Driver Hub.
     *
     * This helps the team:
     * - Check localization
     * - Verify Ivy command switching
     * - Monitor turret angles
     * - Detect unreachable targets
     * - Monitor shooter velocity
     *
     * Telemetry does not directly control mechanisms.
     */

    private void updateTelemetry() {

        Pose pose = follower.pose();


        telemetry.addLine("===== BIOBUZZ IVY V5 =====");

        telemetry.addData(
                "Alliance",
                BLUE_ALLIANCE ? "BLUE" : "RED"
        );


        /*
         * PEDRO LOCALIZATION
         */

        telemetry.addLine("===== PEDRO =====");

        telemetry.addData("Robot X", "%.2f", pose.x());

        telemetry.addData("Robot Y", "%.2f", pose.y());

        telemetry.addData(
                "Heading",
                "%.2f deg",
                Math.toDegrees(pose.heading())
        );


        /*
         * IVY COMMAND STATUS
         */

        telemetry.addLine("===== IVY TURRET =====");

        telemetry.addData(
                "Mode",
                autoAimEnabled ? "AUTO" : "MANUAL"
        );


        /*
         * Shows whether the manual command is executing
         */
        telemetry.addData(
                "Manual Command Running",
                Scheduler.isRunning(manualTurretCommand)
        );


        /*
         * Shows whether auto-aim is executing
         */
        telemetry.addData(
                "Auto Aim Command Running",
                Scheduler.isRunning(autoAimCommand)
        );


        /*
         * TURRET DATA
         */

        telemetry.addData(
                "Turret Position",
                "%.3f",
                turretPosition
        );

        telemetry.addData(
                "Requested Angle",
                "%.2f deg",
                turretAngle
        );

        telemetry.addData(
                "Aim Limited",
                aimLimited ? "YES" : "NO"
        );

        telemetry.addData(
                "HIVE Distance",
                "%.2f",
                distanceToHive
        );


        /*
         * SHOOTER DATA
         */

        telemetry.addLine("===== SHOOTER =====");

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
                shooter.getVelocity()
        );


        /*
         * Send all telemetry to the Driver Hub
         */
        telemetry.update();
    }
}
