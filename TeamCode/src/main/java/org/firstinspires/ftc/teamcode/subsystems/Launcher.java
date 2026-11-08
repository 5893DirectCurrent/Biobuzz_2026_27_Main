package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot.Robot;

public class Launcher {
    public DcMotorEx flywheel;
    public static boolean activated = false;

    public Launcher(Robot robot) {
        flywheel = robot.hardwareMap.get(DcMotorEx.class, "Flywheel");
    }

    private void setPower(double power) {
        flywheel.setPower(power);
    }

    public void on() {
        activated = true;
    }

    public void off() {
        activated = false;
    }

    public Command periodic() {
        return infinite(() -> {
            if(activated) {
                setPower(1);
            } else {
                setPower(0);
            }
        });
    }
}



