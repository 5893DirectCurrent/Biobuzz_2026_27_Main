/*package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot.Robot;

public class Intake {
    public DcMotor intake = null;
    public boolean activated;
    public boolean outtake = false;

    public Intake(Robot robot) {
        intake = robot.hardwareMap.get(DcMotor.class, "Intake");
    }

    public void setPower(double power) {
        intake.setPower(power);
    }

    public void on() {
        activated = true;
    }

    public void off() {
        activated = false;
    }

    public void out() {
        outtake = true;
    }
    public void in() {
        outtake = false;
    }

    public Command periodic() {
        return infinite(() -> {
            if(activated) {
                setPower(1);
            } else {
                setPower(0);
            }

            if(outtake) {
                intake.setDirection(DcMotor.Direction.REVERSE);
            } else {
                intake.setDirection(DcMotor.Direction.FORWARD);
            }
        });
    }
}*/
