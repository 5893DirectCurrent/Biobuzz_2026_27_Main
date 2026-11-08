package org.firstinspires.ftc.teamcode.Robot;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.*;

public class Robot {
    public HardwareMap hardwareMap;
    public Launcher launcher;
    public Drivetrain drivetrain;

    public Robot(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;

        launcher = new Launcher(this);
        drivetrain = new Drivetrain(this);
    }
}

