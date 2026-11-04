package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Drivetrain {
    public DcMotor fLD = null;
    public DcMotor fRD = null;
    public DcMotor bLD = null;
    public DcMotor bRD = null;

    public static Command forward() {
        return Command.build()
                .setExecute()



    }
}
