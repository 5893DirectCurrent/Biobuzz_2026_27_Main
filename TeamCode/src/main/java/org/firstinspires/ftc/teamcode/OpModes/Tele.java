package org.firstinspires.ftc.teamcode.OpModes;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.pedropathing.ivy.behaviors.EndCondition;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotOpMode;

@TeleOp(name="TeleOp", group="Linear OpMode")
public class Tele extends RobotOpMode {


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

        if(gamepad1.left_bumper) {
            robot.drivetrain.drivePower = .5;
        } else {
            robot.drivetrain.drivePower = 1;
        }

        if(gamepad2.a) {
            robot.launcher.on();
        } else {
            robot.launcher.off();
        }

        if(gamepad1.left_bumper) {
            robot.drivetrain.drivePower = robot.drivetrain.slow;
        } else {
            robot.drivetrain.drivePower = 1;
        }

        //Pinpoint
        /*if(gamepad1.y) {
            robot.drivetrain.pinpointOdometry.setHeading(0, AngleUnit.DEGREES);
        }*/

        //IMU
        if(gamepad1.y) {
            robot.drivetrain.imu.resetYaw();
        }

        if(gamepad1.share) {
            robot.drivetrain.RobotCentric = true;
        }

        super.loop();
    }
}