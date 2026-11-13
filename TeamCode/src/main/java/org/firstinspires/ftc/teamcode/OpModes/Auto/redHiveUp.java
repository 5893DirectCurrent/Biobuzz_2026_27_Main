package org.firstinspires.ftc.teamcode.OpModes.Auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import static java.lang.Thread.sleep;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotOpMode;

@Autonomous(name="RedHiveUp")
public class redHiveUp extends RobotOpMode {
    Robot robot;
    public PoseFactory poseFactory = PoseFactory.degrees();
    Follower follower;
    Pose startPose = poseFactory.of(58.75, 7.75, 90);
    Pose park = poseFactory.of(13.75, 94.75, 270);
    Pose parkCurve = poseFactory.of(19.75, 46.75, 0);

    private Path park() {
        return curve(startPose, parkCurve, park).linear(startPose, park);
    }

    public Command auto() {
        return sequential(
                //robot.intake.on(),
                follow(follower, park())
        );
    }

    @Override
    public void init() {
        follower.setPose(startPose);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        follower.update();

        super.loop();
    }
}
