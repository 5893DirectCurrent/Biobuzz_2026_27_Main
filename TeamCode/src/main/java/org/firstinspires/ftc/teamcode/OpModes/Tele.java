package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot.Robot;

public class Tele extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() {
        robot.drivetrain.fieldCentric();

        if(gamepad2.a) {
            robot.launcher.on();
        }
        else {
            robot.launcher.off();
        }

        if(gamepad1.left_bumper) {
            robot.drivetrain.max = robot.drivetrain.slow;
        }
        else {
            robot.drivetrain.max = 1;
        }

        if(gamepad1.y) {
            robot.drivetrain.pinpointOdometry.setHeading(0, AngleUnit.DEGREES);
        }


        telemetry.addData("Shooting:", robot.launcher.activated);
    }

    /*@Override
    public void loop() {

        robot.drivetrain.fieldCentric();

        if(gamepad2.a) {
            robot.launcher.on();
        }
        else {
            robot.launcher.off();
        }

        if(gamepad1.left_bumper) {
            robot.drivetrain.max = robot.drivetrain.slow;
        }
        else {
            robot.drivetrain.max = 1;
        }

        if(gamepad1.y) {
            robot.drivetrain.pinpointOdometry.setHeading(0, AngleUnit.DEGREES);
        }


        telemetry.addData("Shooting:", robot.launcher.activated);
    }*/
}