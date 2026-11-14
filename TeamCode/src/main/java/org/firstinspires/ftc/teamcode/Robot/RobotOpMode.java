package org.firstinspires.ftc.teamcode.Robot;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public abstract class RobotOpMode extends OpMode {
    protected Robot robot;

    @Override
    public void init() {
        Scheduler.reset();
        
        robot = new Robot(this);
        schedule(
                robot.launcher.periodic()
                //robot.intake.periodic(),
                //robot.shootPoseRed.periodic(),
                //robot.shootPoseBlue.periodic()
        );
    }

    @Override
    public void init_loop() {
        Scheduler.execute();
        robot.drivetrain.pinpointOdometry.update();
    }

    @Override
    public void loop() {
        Scheduler.execute();

        if(!robot.drivetrain.RobotCentric) {
            telemetry.addData("FrontLeftPower", robot.drivetrain.frontLeftSpeedF);
            telemetry.addData("FrontRightPower", robot.drivetrain.frontRightSpeedF);
            telemetry.addData("BackLeftPower", robot.drivetrain.backLeftSpeedF);
            telemetry.addData("BackRightPower", robot.drivetrain.backRightSpeedF);
        } else {
            telemetry.addData("FrontLeftPower", robot.drivetrain.frontLeftSpeedR);
            telemetry.addData("FrontRightPower", robot.drivetrain.frontRightSpeedR);
            telemetry.addData("BackLeftPower", robot.drivetrain.backLeftSpeedR);
            telemetry.addData("BackRightPower", robot.drivetrain.backRightSpeedR);
        }
        telemetry.addData("X cord", robot.drivetrain.pinpointOdometry.getEncoderX());
        telemetry.addData("X cord", robot.drivetrain.pinpointOdometry.getEncoderY());
        telemetry.addData("X cord", robot.drivetrain.pinpointOdometry.getHeading(AngleUnit.DEGREES));
        telemetry.update();
        robot.drivetrain.pinpointOdometry.update();
    }
}
