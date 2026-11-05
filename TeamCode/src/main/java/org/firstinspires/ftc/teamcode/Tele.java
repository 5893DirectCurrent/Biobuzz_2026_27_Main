package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.EndCondition;

import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import static org.firstinspires.ftc.teamcode.subsystems.Launcher.flywheel;

public class Tele extends LinearOpMode {

    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");



        waitForStart();
        driver

        Launcher.shoot().schedule();

        while (opModeIsActive()) {


            if(gamepad2.a) {
                Launcher.shoot().execute();
            }
            else {
                Launcher.shoot().execute();
            }
        }
    }
}