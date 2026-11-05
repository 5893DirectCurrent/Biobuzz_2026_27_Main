package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.pedropathing.revhub.localizers.RevHubIMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Drivetrain {

    public DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive = null;
    public double max = 1;
    public double slow = .5;
    public double power = max;

    double forward = -gamepad1.left_stick_y;
    double strafe = gamepad1.left_stick_x;
    double turn = gamepad1.right_stick_x;

    //Field Centric
    public GoBildaPinpointDriver pinpointOdometry;
    public IMU imu;
    public boolean RobotCentric = true;

    //Hardware mapping
    public void init(HardwareMap map) {
        //Map drive motors
        frontLeftDrive = map.get(DcMotor.class, "frontLeftMotor");
        frontRightDrive = map.get(DcMotor.class, "frontRightMotor");
        backLeftDrive = map.get(DcMotor.class, "backLeftMotor");
        backRightDrive = map.get(DcMotor.class, "backRightMotor");

        pinpointOdometry = map.get(GoBildaPinpointDriver.class, "pinpoint");
        imu = map.get(IMU.class, "IMU");

        //Set motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public void robotCentric() {
        double frontLeftSpeedR = forward + strafe + turn;
        double frontRightSpeedR = forward - strafe - turn;
        double backLeftSpeedR = forward - strafe + turn;
        double backRightSpeedR = forward + strafe - turn;

        if(frontLeftSpeedR > max) {
            frontLeftSpeedR = max;
        }
        if(frontRightSpeedR > max) {
            frontRightSpeedR = max;
        }
        if(backLeftSpeedR > max) {
            backLeftSpeedR = max;
        }
        if(backRightSpeedR > max) {
            backRightSpeedR = max;
        }

        if(RobotCentric) {
            frontLeftDrive.setPower(frontLeftSpeedR);
            frontRightDrive.setPower(frontRightSpeedR);
            backLeftDrive.setPower(backLeftSpeedR);
            backRightDrive.setPower(backRightSpeedR);
        }
    }


    public void fieldCentric() {
        //Math
        Pose2D currentPos = pinpointOdometry.getPosition();
        double heading = currentPos.getHeading(AngleUnit.RADIANS);
        double cosAngle = Math.cos((Math.PI / 2)-heading);
        double sinAngle = Math.sin((Math.PI / 2)-heading);
        double fieldStrafe = -forward * sinAngle + strafe * cosAngle;
        double fieldForward = forward * cosAngle + strafe * sinAngle;

        double frontLeftSpeedF = fieldForward + fieldStrafe + turn;
        double frontRightSpeedF = fieldForward - fieldStrafe - turn;
        double backLeftSpeedF = fieldForward - fieldStrafe + turn;
        double backRightSpeedF = fieldForward + fieldStrafe - turn;

        if(frontLeftSpeedF > max) {
            frontLeftSpeedF = max;
        }
        if(frontRightSpeedF > max) {
            frontRightSpeedF = max;
        }
        if(backLeftSpeedF > max) {
            backLeftSpeedF = max;
        }
        if(backRightSpeedF > max) {
            backRightSpeedF = max;
        }

        if(!RobotCentric) {
            frontLeftDrive.setPower(frontLeftSpeedF);
            frontRightDrive.setPower(frontRightSpeedF);
            backLeftDrive.setPower(backLeftSpeedF);
            backRightDrive.setPower(backRightSpeedF);
        }
    }
}
