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
    public DcMotorEx flywheelL;
    public DcMotorEx flywheelR;
    public static boolean activated = false;

    public Launcher(Robot robot) {
        flywheelL = robot.hardwareMap.get(DcMotorEx.class, "FlywheelLeft");
        flywheelR = robot.hardwareMap.get(DcMotorEx.class, "FlywheelRight");
    }

    private void setPower(double power) {
        flywheelL.setPower(power);
        flywheelR.setPower(power);
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



