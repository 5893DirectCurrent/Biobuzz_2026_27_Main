package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot.RobotOpMode;

@TeleOp(name="blueSide", group="Linear OpMode")
public class TeleOpBlue extends RobotOpMode {
    @Override
    public void init() {
        super.init();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        if(robot.drivetrain.RobotCentric)
            robot.drivetrain.robotCentric(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

        if(!robot.drivetrain.RobotCentric)
            robot.drivetrain.fieldCentric(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

        //Gamepad1/Driver
        if(gamepad1.left_bumper) {
            robot.drivetrain.drivePower = robot.drivetrain.slow;
        } else {
            robot.drivetrain.drivePower = 1;
        }

        /*if(gamepad1.right_trigger > 0.5) {
            robot.intake.on();
        } else {
            robot.intake.off();
        }

        if(gamepad1.left_trigger > 0.5) {
            robot.intake.out();
            robot.intake.on();
        } else {
            robot.intake.off();
            robot.intake.in();
        }*/

        //Pinpoint
        if(gamepad1.y) {
            robot.drivetrain.pinpointOdometry.setHeading(0, AngleUnit.DEGREES);
        }

        //IMU
        /*if(gamepad1.y) {
            robot.drivetrain.imu.resetYaw();
        }*/

        if(gamepad1.share) {
            robot.drivetrain.RobotCentric = true;
        }

        if(gamepad1.dpad_left) {
            robot.shootPoseBlue.runToLeft();
        } else robot.shootPoseBlue.notShootingLeft();
        if(gamepad1.dpad_right) {
            robot.shootPoseBlue.runToRight();
        } else {
            robot.shootPoseBlue.notShootingRight();
        }

        //Gamepad2/Operator
        if(gamepad2.a) {
            robot.launcher.on();
        } else {
            robot.launcher.off();
        }

        super.loop();
    }
}
