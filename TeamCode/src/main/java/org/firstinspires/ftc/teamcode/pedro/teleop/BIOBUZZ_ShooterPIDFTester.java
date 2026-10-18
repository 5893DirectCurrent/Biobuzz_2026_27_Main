package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "BIOBUZZ Shooter PIDF Tester", group = "Testing")
public class BIOBUZZ_ShooterPIDFTester extends LinearOpMode {

    // ============================================================
    // HARDWARE
    // ============================================================

    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
    private DcMotor intake;
    private DcMotor LF, LB, RF, RB;

    /* Intake is controlled independently of the two shooter motors. */
    public static double INTAKE_POWER = 1;
    public static double TRIGGER_DEADBAND = 0.15;
    public static double DRIVE_SPEED_LIMIT = 0.80; /* Initial testing speed limit. */

    // ============================================================
    // STARTING PIDF VALUES
    // ============================================================

    public static double P = 1.0000;
    public static double I = 0.0000;
    public static double D = 0.0000;
    public static double F = 12.0000;

    // ============================================================
    // TEST SETTINGS
    // ============================================================

    public static double TARGET_VELOCITY = 1800;

    // How much each button press changes the value
    public static double P_INCREMENT = 0.0001;
    public static double I_INCREMENT = 0.00001;
    public static double D_INCREMENT = 0.1;
    public static double F_INCREMENT = 0.5;

    // ============================================================
    // BUTTON STATE
    // ============================================================

    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    private boolean shooterRunning = false;
    private boolean lastStart = false;
    private boolean lastLeftStickButton = false;

    @Override
    public void runOpMode() {

        // ========================================================
        // HARDWARE
        // ========================================================

        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        /* Mecanum drivetrain, copied from Robot_Centric_Drive.java. */
        LF = hardwareMap.get(DcMotor.class, "LF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        RF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.REVERSE);
        LF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.FORWARD);
        for (DcMotor motor : new DcMotor[]{LF, LB, RF, RB}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
        }

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);

        /* Verify directions so both wheels propel the game piece correctly. */
        shooterLeft.setDirection(DcMotorEx.Direction.FORWARD);
        shooterRight.setDirection(DcMotorEx.Direction.REVERSE);

        shooterLeft.setMode(
                com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER
        );

        shooterRight.setMode(
                com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER
        );

        shooterLeft.setZeroPowerBehavior(
                com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT
        );

        shooterRight.setZeroPowerBehavior(
                com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT
        );

        /* Nonzero initial feedforward lets the velocity controller spin up.
         * These are starting estimates, not tuned PIDF coefficients. */
        applyPIDF();

        telemetry.addLine("BIOBUZZ SHOOTER PIDF TESTER");
        telemetry.addLine("---------------------------");
        telemetry.addLine("");
        telemetry.addLine("A / B     = Change P");
        telemetry.addLine("X / Y     = Change F");
        telemetry.addLine("D-PAD     = Change target velocity");
        telemetry.addLine("Bumpers   = Change I");
        telemetry.addLine("START or L3 = Shooter ON/OFF");
        telemetry.addLine("Left stick = Drive / strafe");
        telemetry.addLine("Right stick X = Rotate");
        telemetry.addLine("RT hold = Intake forward");
        telemetry.addLine("LT hold = Intake reverse");
        telemetry.addLine("");
        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        /* Require a fresh button press after START. */
        lastStart = gamepad1.start;
        lastLeftStickButton = gamepad1.left_stick_button;

        try {
            while (opModeIsActive()) {

                // ====================================================
                // PIDF ADJUSTMENT
                // ====================================================

                adjustPIDF();
                driveRobot();

                /* START or left-stick click toggles both motors.
                 * L3 is an alternative if START is reserved on the controller. */
                if ((gamepad1.start && !lastStart)
                        || (gamepad1.left_stick_button && !lastLeftStickButton)) {
                    shooterRunning = !shooterRunning;
                }
                lastStart = gamepad1.start;
                lastLeftStickButton = gamepad1.left_stick_button;

                // ====================================================
                // SHOOTER
                // ====================================================

                if (shooterRunning) {

                    shooterLeft.setVelocity(TARGET_VELOCITY);
                    shooterRight.setVelocity(TARGET_VELOCITY);

                } else {

                    shooterLeft.setVelocity(0);
                    shooterRight.setVelocity(0);
                }

                /* Hold RT to intake, LT to reverse, release to stop.
                 * Reverse takes priority if both triggers are held. */
                double intakePower = 0;
                if (gamepad1.left_trigger > TRIGGER_DEADBAND) {
                    intakePower = -INTAKE_POWER;
                } else if (gamepad1.right_trigger > TRIGGER_DEADBAND) {
                    intakePower = INTAKE_POWER;
                }
                intake.setPower(intakePower);

                // ====================================================
                // TELEMETRY
                // ====================================================

                showTelemetry();
            }
        } finally {
            shooterLeft.setVelocity(0);
            shooterRight.setVelocity(0);
            intake.setPower(0);
            LF.setPower(0);
            LB.setPower(0);
            RF.setPower(0);
            RB.setPower(0);
        }
    }

    // ============================================================
    // APPLY PIDF
    // ============================================================

    /* Robot-centric mecanum drive from the supplied drivetrain file.
     * Speed is limited for early shooter testing; sticks remain unchanged. */
    private void driveRobot() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        LF.setPower(DRIVE_SPEED_LIMIT * (y + x + rx) / denominator);
        LB.setPower(DRIVE_SPEED_LIMIT * (y - x + rx) / denominator);
        RF.setPower(DRIVE_SPEED_LIMIT * (y - x - rx) / denominator);
        RB.setPower(DRIVE_SPEED_LIMIT * (y + x - rx) / denominator);
    }

    private void applyPIDF() {

        shooterLeft.setVelocityPIDFCoefficients(P, I, D, F);
        shooterRight.setVelocityPIDFCoefficients(P, I, D, F);
    }

    // ============================================================
    // PIDF CONTROLS
    // ============================================================

    private void adjustPIDF() {

        // --------------------------------------------------------
        // P
        // --------------------------------------------------------

        if (gamepad1.a && !lastA) {

            P += P_INCREMENT;
            applyPIDF();
        }

        if (gamepad1.b && !lastB) {

            P -= P_INCREMENT;

            if (P < 0) {
                P = 0;
            }

            applyPIDF();
        }

        // --------------------------------------------------------
        // F
        // --------------------------------------------------------

        if (gamepad1.x && !lastX) {

            F += F_INCREMENT;
            applyPIDF();
        }

        if (gamepad1.y && !lastY) {

            F -= F_INCREMENT;

            if (F < 0) {
                F = 0;
            }

            applyPIDF();
        }

        // --------------------------------------------------------
        // I
        // --------------------------------------------------------

        if (gamepad1.left_bumper && !lastLeftBumper) {

            I += I_INCREMENT;
            applyPIDF();
        }

        if (gamepad1.right_bumper && !lastRightBumper) {

            I -= I_INCREMENT;

            if (I < 0) {
                I = 0;
            }

            applyPIDF();
        }

        // --------------------------------------------------------
        // TARGET VELOCITY
        // --------------------------------------------------------

        if (gamepad1.dpad_up && !lastDpadUp) {

            TARGET_VELOCITY += 50;
        }

        if (gamepad1.dpad_down && !lastDpadDown) {

            TARGET_VELOCITY -= 50;

            if (TARGET_VELOCITY < 0) {
                TARGET_VELOCITY = 0;
            }
        }

        // --------------------------------------------------------
        // INCREMENT SIZE
        // --------------------------------------------------------

        if (gamepad1.dpad_right && !lastDpadRight) {

            P_INCREMENT *= 2;
            I_INCREMENT *= 2;
            D_INCREMENT *= 2;
            F_INCREMENT *= 2;
        }

        if (gamepad1.dpad_left && !lastDpadLeft) {

            P_INCREMENT /= 2;
            I_INCREMENT /= 2;
            D_INCREMENT /= 2;
            F_INCREMENT /= 2;
        }

        // ========================================================
        // SAVE BUTTON STATES
        // ========================================================

        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;

        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;

        lastLeftBumper = gamepad1.left_bumper;
        lastRightBumper = gamepad1.right_bumper;
    }

    // ============================================================
    // TELEMETRY
    // ============================================================

    private void showTelemetry() {

        double leftVelocity = shooterLeft.getVelocity();
        double rightVelocity = shooterRight.getVelocity();

        double leftError = TARGET_VELOCITY - leftVelocity;
        double rightError = TARGET_VELOCITY - rightVelocity;

        telemetry.addLine("===== PIDF TESTER =====");

        telemetry.addLine("");

        telemetry.addData(
                "Shooter",
                shooterRunning ? "RUNNING" : "OFF"
        );

        telemetry.addData(
                "Target Velocity",
                "%.0f",
                TARGET_VELOCITY
        );

        telemetry.addData("Left Velocity", "%.0f", leftVelocity);
        telemetry.addData("Right Velocity", "%.0f", rightVelocity);
        telemetry.addData("Left Error", "%.0f", leftError);
        telemetry.addData("Right Error", "%.0f", rightError);
        telemetry.addData("Intake power", "%.2f", intake.getPower());
        telemetry.addData("Drive speed limit", "%.0f%%", DRIVE_SPEED_LIMIT * 100);

        telemetry.addLine("");

        telemetry.addLine("PIDF VALUES");

        telemetry.addData("P", "%.7f", P);
        telemetry.addData("I", "%.7f", I);
        telemetry.addData("D", "%.7f", D);
        telemetry.addData("F", "%.7f", F);
        telemetry.addData("Toggle input", "START / L3");

        telemetry.addLine("");

        telemetry.addLine("INCREMENTS");

        telemetry.addData("P Step", "%.7f", P_INCREMENT);
        telemetry.addData("I Step", "%.7f", I_INCREMENT);
        telemetry.addData("D Step", "%.7f", D_INCREMENT);
        telemetry.addData("F Step", "%.7f", F_INCREMENT);

        telemetry.addLine("");

        telemetry.addLine("CONTROLS");
        telemetry.addLine("Left stick = Drive / strafe");
        telemetry.addLine("Right stick X = Rotate");
        telemetry.addLine("A/B = P +/-");
        telemetry.addLine("X/Y = F +/-");
        telemetry.addLine("LB/RB = I +/-");
        telemetry.addLine("DPad Up/Down = Target");
        telemetry.addLine("DPad Left/Right = Step Size");
        telemetry.addLine("START or L3 = Both shooters ON/OFF");
        telemetry.addLine("RT = Intake forward (hold)");
        telemetry.addLine("LT = Intake reverse (hold)");

        telemetry.update();
    }
}