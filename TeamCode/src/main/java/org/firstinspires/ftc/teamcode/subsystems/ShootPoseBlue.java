package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class ShootPoseBlue {
    Robot robot;
    public PoseFactory poseFactory = PoseFactory.degrees();
    public Follower follower;
    public Pose startPoseTele = poseFactory.of(
            robot.drivetrain.pinpointOdometry.getEncoderX(),
            robot.drivetrain.pinpointOdometry.getEncoderY(),
            robot.drivetrain.pinpointOdometry.getHeading(AngleUnit.DEGREES)
    );
    public Pose shootPoseTeleL = poseFactory.of(76.75, 10.75, 90);
    public Pose shootPoseTeleR = poseFactory.of(82.75, 130.75, 90);
    public Pose shootCurveTeleL = poseFactory.of(robot.drivetrain.pinpointOdometry.getEncoderX(), 130.75, 0);
    public Pose shootCurveTeleR = poseFactory.of(robot.drivetrain.pinpointOdometry.getEncoderX(), 10.75, 0);
    public boolean goingLeft = false;
    public boolean goingRight = false;
    public void init(Robot robot) {
        follower = Constants.create(robot.hardwareMap); {
        }
    }

    public Path goShootLeft() {
        return curve(startPoseTele, shootCurveTeleL, shootPoseTeleL).linear(startPoseTele, shootCurveTeleL);
    }
    public Path goShootRight() {
        return curve(startPoseTele, shootCurveTeleR, shootPoseTeleR).linear(startPoseTele, shootPoseTeleR);
    }

    public void runToLeft() {
        goingLeft = true;
    }
    public void runToRight() {
        goingRight = true;
    }

    public void notShootingLeft() {
        goingLeft = false;
    }
    public void notShootingRight() {
        goingRight = false;
    }

    public Command periodic() {
        return infinite(() -> {
            if (goingLeft) {
                follow(follower, goShootLeft());
            }
            if (goingRight) {
                follow(follower, goShootRight());
            }
            if(!goingLeft && !goingRight) {
                follower.stop();
            }
        });
    }
}
