/*package org.firstinspires.ftc.robotcontroller.test;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;

public class launchCommand {
    public static Command launcherSwivel(CRServo shootServo) {
        return Command.build()
                .setExecute(() -> shootServo.setPower(1))
                .setEnd(EndCondition -> shootServo.setPower(0));
    }
}*/