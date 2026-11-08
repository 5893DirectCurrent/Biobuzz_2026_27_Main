package org.firstinspires.ftc.teamcode.Robot;

import static com.pedropathing.ivy.Scheduler.schedule;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Launcher;

public abstract class RobotOpMode extends OpMode {
    protected Robot robot;

    @Override
    public void init() {
        
        robot = new Robot(this);
        schedule(
                robot.launcher.periodic()
                //robot.drivetrain.periodic()
        );
    }

    @Override
    public void init_loop() {
        Scheduler.execute();
    }

    @Override
    public void loop() {
        Scheduler.execute();

        telemetry.addData("FrontLeftPower", robot.drivetrain.frontLeftSpeedF);
        telemetry.addData("FrontRightPower", robot.drivetrain.frontRightSpeedF);
        telemetry.addData("BackLeftPower", robot.drivetrain.backLeftSpeedF);
        telemetry.addData("BackRightPower", robot.drivetrain.backRightSpeedF);
        telemetry.update();
    }
}
