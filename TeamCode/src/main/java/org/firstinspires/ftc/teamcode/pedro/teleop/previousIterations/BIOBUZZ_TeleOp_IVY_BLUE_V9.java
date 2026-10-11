
package org.firstinspires.ftc.teamcode.pedro.teleop.previousIterations;

/*
 * BIOBUZZ IVY V9: BLUE MATCH TELEOP
 * Fixed starting pose until AprilTag field mapping is verified.
 * Mirrored target: X = 2 * REFERENCE_X - live robot X; Y = 72.
 * Target Z is reserved for later shooter trajectory calculations.
 * Limelight initialization and ~30-second corrections remain COMMENTED OUT.
 * Driver controls are unchanged from V5/V8.
 * NOTE: Verify Ivy API and servo calibration on the actual robot.
 */


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
/*
 * FUTURE LIMELIGHT IMPORTS (intentionally disabled)
 * After AprilTag mapping and SDK verification, enable the imports needed for:
 *   com.qualcomm.hardware.limelightvision.Limelight3A
 *   com.qualcomm.hardware.limelightvision.LLResult
 * Do not enable until the installed FTC SDK supports these types.
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


@TeleOp(name = "BIOBUZZ IVY BLUE V9")
public class BIOBUZZ_TeleOp_IVY_BLUE_V9 extends LinearOpMode {


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
     * Alliance is fixed by this OpMode file; select RED or BLUE on Driver Hub.
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

    private static final double REFERENCE_X = 86.0;
    private static final double TARGET_Y = 72.0;
    private static final double TARGET_Z_IN = 0.0; /* TODO: measured target height */

    /* Starting pose is temporary until AprilTag localization is ready.
       Set these to the robot's actual placement and heading before a run. */
    private static final double START_X_IN = 24.0;
    private static final double START_Y_IN = 24.0;
    private static final double START_HEADING_DEG = 0.0;

    /* Turret offset from Pedro robot origin, in robot-local inches.
       X forward, Y left; calibrate before relying on auto-aim. */
    private static final double TURRET_OFFSET_X_IN = 0.0;
    private static final double TURRET_OFFSET_Y_IN = 0.0;
    private static final double TURRET_MOUNT_ZERO_DEG = 0.0;
    private static final double TURRET_DEGREES_PER_SERVO_UNIT = 180.0;
    private static final double TURRET_MAX_ANGLE_DEG = 90.0;


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

    /*
     * FUTURE LIMELIGHT STATE (disabled)
     * Store the Limelight3A device, the last accepted field pose,
     * the timestamp of the last correction attempt, and localization status.
     * Example configuration: correction interval = 30_000 milliseconds.
     * Use a monotonic FTC timer (e.g. ElapsedTime) when implemented.
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
            poseFactory.of(START_X_IN, START_Y_IN, START_HEADING_DEG);


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
        /*
         * FUTURE LIMELIGHT HARDWARE SETUP (disabled)
         * 1. Obtain the Limelight3A from hardwareMap using its actual name.
         * 2. Select the verified AprilTag pipeline and start the camera.
         * 3. Confirm the robot-to-camera transform and field map.
         * 4. Read LLResult and reject invalid / stale pose estimates.
         * This must not interfere with existing Pedro initialization.
         */
        follower = Constants.create(hardwareMap);

        /*
         * Tell Pedro where the robot starts
         */
        /*
         * FUTURE INITIAL APRILTAG LOCALIZATION (disabled)
         * For now, startPose remains the active fixed-coordinate fallback.
         * When the pipeline is ready, acquire a valid field-relative pose,
         * transform its units/origin/axes/heading into Pedro coordinates,
         * and initialize Pedro from that pose instead of startPose.
         * Never assume a visible tag alone guarantees a usable robot pose.
         */
        follower.setPose(startPose);


        /*
         * CREATE IVY COMMANDS
         */

        createCommands();


        telemetry.addLine("BIOBUZZ IVY BLUE V9 READY");

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
                /*
                 * FUTURE PERIODIC LIMELIGHT RELOCALIZATION (disabled)
                 * After follower.update(), approximately every 30 seconds:
                 *   - Request the newest AprilTag robot-pose estimate.
                 *   - Check target validity, freshness, ambiguity, and pose quality.
                 *   - Convert Limelight coordinates to Pedro's field frame.
                 *   - Reject impossible jumps and inconsistent heading.
                 *   - Apply an accepted correction via the verified Pedro API.
                 *   - Record the attempt time even when no tag is visible.
                 * Continue Pedro odometry between accepted corrections.
                 * Consider motion/latency compensation before using setPose.
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
        /* Dynamic virtual target: mirror robot X across alliance reference X.
           RED: (116 - robotX, 72, Z); BLUE: (172 - robotX, 72, Z).
           TARGET_Z_IN is retained for future shooter elevation calculations.
           This command only controls the turret's horizontal direction. */
        Pose pose = follower.pose();
        double robotX = pose.x();
        double robotY = pose.y();
        double heading = pose.heading();
        double targetX = 2.0 * REFERENCE_X - robotX;
        double targetY = TARGET_Y;

        /* Convert turret's robot-local mounting offset to field coordinates. */
        double turretX = robotX
                + Math.cos(heading) * TURRET_OFFSET_X_IN
                - Math.sin(heading) * TURRET_OFFSET_Y_IN;
        double turretY = robotY
                + Math.sin(heading) * TURRET_OFFSET_X_IN
                + Math.cos(heading) * TURRET_OFFSET_Y_IN;

        double dx = targetX - turretX;
        double dy = targetY - turretY;
        distanceToHive = Math.hypot(dx, dy);
        double fieldAngle = Math.atan2(dy, dx);
        angleToHive = Math.toDegrees(fieldAngle);
        double relativeAngle = fieldAngle - heading
                - Math.toRadians(TURRET_MOUNT_ZERO_DEG);
        relativeAngle = Math.atan2(Math.sin(relativeAngle), Math.cos(relativeAngle));
        turretAngle = Math.toDegrees(relativeAngle);

        /* Do not wrap an unreachable rearward target into a reachable angle. */
        double limitedAngle = Math.max(-TURRET_MAX_ANGLE_DEG,
                Math.min(TURRET_MAX_ANGLE_DEG, turretAngle));
        double requestedPosition = TURRET_CENTER
                + TURRET_DIRECTION * limitedAngle / TURRET_DEGREES_PER_SERVO_UNIT;
        aimLimited = Math.abs(turretAngle - limitedAngle) > 0.01
                || requestedPosition < TURRET_MIN
                || requestedPosition > TURRET_MAX;
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

    /* Driver-facing telemetry: keep the match display easy to scan. */
    private void updateTelemetry() {
        Pose pose = follower.pose();
        telemetry.addLine("BIOBUZZ BLUE | IVY V9");
        telemetry.addData("Turret", autoAimEnabled ? "AUTO" : "MANUAL");
        telemetry.addData("Aim limit", aimLimited ? "LIMITED" : "OK");
        telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
        telemetry.addData("Shooter target", "%.0f ticks/s", shooterTarget);
        telemetry.addData("Shooter actual", "%.0f ticks/s", shooter.getVelocity());
        telemetry.addData("Position (in)", "%.1f, %.1f", pose.x(), pose.y());
        telemetry.addData("Heading", "%.1f deg", Math.toDegrees(pose.heading()));
        telemetry.addData("Virtual target (in)", "%.1f, %.1f, %.1f",
                2.0 * REFERENCE_X - pose.x(), TARGET_Y, TARGET_Z_IN);
        telemetry.addData("Localization", "FIXED START / PEDRO");
        /* FUTURE LIMELIGHT STATUS (disabled): show last accepted tag pose,
           age of last correction, and whether auto-aim localization is valid. */
        telemetry.update();
    }
}
