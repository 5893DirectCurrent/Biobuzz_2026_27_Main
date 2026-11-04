package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Launcher {
    public static DcMotor flywheel =  null;



    public static Command shoot() {
        return Command.build()
                .setExecute(() -> flywheel.setPower(1))
                .setEnd(endCondition -> flywheel.setPower(0));
    }

}
