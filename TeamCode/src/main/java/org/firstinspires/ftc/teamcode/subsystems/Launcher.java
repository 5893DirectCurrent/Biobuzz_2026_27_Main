package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Launcher {
    public static DcMotor flywheel = null;
    public boolean activated = false;

    public void init(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotor.class, "Flywheel");
    }

    public void off() {
        activated = false;
        flywheel.setPower(0);
    }
    public void on() {
        activated = true;
    }

    public void shoot() {
        if (activated)
                flywheel.setPower(1);
    }

    public Command toggle() {
        return Commands.instant(this::shoot);
    }


}



