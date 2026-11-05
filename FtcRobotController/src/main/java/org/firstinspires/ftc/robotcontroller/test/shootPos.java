/*package org.firstinspires.ftc.robotcontroller.test;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.Robot.Robot;

public class shootPos {
    //Robot robot;
    double currentX = robot.drivetrain.pinpointOdometry.getPosX(DistanceUnit.INCH);
    double currentY = robot.drivetrain.pinpointOdometry.getPosY(DistanceUnit.INCH);
    double heading = robot.drivetrain.pinpointOdometry.getHeading(AngleUnit.RADIANS);
    protected Pose currentPos = new Pose(currentX, currentY, heading);
    protected Pose shootPos = new Pose(56, 36, Math.toRadians(0));
    public Follower getFollower() {
        return;
    }

    public static Command runToShoot() {
        return Command.build()
                .setExecute(
                        follow(robot.getFollower(), goToShoot)
                );
    }

    public void buildPaths() {
        goToShoot = getFollower().pathBuilder()
                .addPath
    }

}*/
