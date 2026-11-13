package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.*;

public class Robot {
    public HardwareMap hardwareMap;
    public Launcher launcher;
    public Intake intake;
    public Drivetrain drivetrain;
    public ShootPoseRed shootPoseRed;
    public ShootPoseBlue shootPoseBlue;

    public Robot(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;

        launcher = new Launcher(this);
        intake = new Intake(this);
        drivetrain = new Drivetrain(this);
    }
}

