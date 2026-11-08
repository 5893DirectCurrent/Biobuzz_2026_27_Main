package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.ivy.Command;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot.Robot;

public class Drivetrain {

    public DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    public double frontLeftSpeedR, frontRightSpeedR, backLeftSpeedR, backRightSpeedR;
    public double frontLeftSpeedF, frontRightSpeedF, backLeftSpeedF, backRightSpeedF;
    public double drivePower = 1;
    public double max;
    public double slow = .75;

    //Field Centric
    public GoBildaPinpointDriver pinpointOdometry;
    public NavxMicroNavigationSensor navxOdometry;
    public IMU imu;
    public boolean RobotCentric = false;

    //Hardware mapping
    public Drivetrain(Robot robot) {
        //Map drive motors
        frontLeftDrive = robot.hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightDrive = robot.hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftDrive = robot.hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightDrive = robot.hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //pinpointOdometry = robot.hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");
        //navxOdometry = robot.hardwareMap.get(NavxMicroNavigationSensor.class, "Navx");
        imu = robot.hardwareMap.get(IMU.class, "IMU");

        //Set motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public void robotCentric(double forward, double strafe, double turn) {
        frontLeftSpeedR = forward + strafe + turn;
        frontRightSpeedR = forward - strafe - turn;
        backLeftSpeedR = forward - strafe + turn;
        backRightSpeedR = forward + strafe - turn;

        max = Math.max(Math.abs(frontLeftSpeedR), Math.abs(frontRightSpeedR));
        max = Math.max(max, Math.abs(backLeftSpeedR));
        max = Math.max(max, Math.abs(backRightSpeedR));

        if(max > 1) {
            frontLeftSpeedR /= max;
            frontRightSpeedR /= max;
            backLeftSpeedR /= max;
            backRightSpeedR /= max;
        }

        if(RobotCentric) {
            frontLeftDrive.setPower(frontLeftSpeedR * drivePower);
            frontRightDrive.setPower(frontRightSpeedR * drivePower);
            backLeftDrive.setPower(backLeftSpeedR * drivePower);
            backRightDrive.setPower(backRightSpeedR * drivePower);
        }
    }


    public void fieldCentric(double forward, double strafe, double turn) {
        //Math
        //Pinpoint
        /*Pose2D currentPos = pinpointOdometry.getPosition();
        double heading = currentPos.getHeading(AngleUnit.RADIANS);
        double cosAngle = Math.cos((Math.PI / 2)-heading);
        double sinAngle = Math.sin((Math.PI / 2)-heading);
        double fieldStrafe = -forward * sinAngle + strafe * cosAngle;
        double fieldForward = forward * cosAngle + strafe * sinAngle;*/

        //IMU
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);
        theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        double fieldForward = r * Math.sin(theta);
        double fieldStrafe = r * Math.cos(theta);


        frontLeftSpeedF = fieldForward + fieldStrafe + turn;
        frontRightSpeedF = fieldForward - fieldStrafe - turn;
        backLeftSpeedF = fieldForward - fieldStrafe + turn;
        backRightSpeedF = fieldForward + fieldStrafe - turn;

        max = Math.max(Math.abs(frontLeftSpeedF), Math.abs(frontRightSpeedF));
        max = Math.max(max, Math.abs(backLeftSpeedF));
        max = Math.max(max, Math.abs(backRightSpeedF));

        if(max > 1) {
            frontLeftSpeedF /= max;
            frontRightSpeedF /= max;
            backLeftSpeedF /= max;
            backRightSpeedF /= max;
        }

        if(!RobotCentric) {
            frontLeftDrive.setPower(frontLeftSpeedF);
            frontRightDrive.setPower(frontRightSpeedF);
            backLeftDrive.setPower(backLeftSpeedF);
            backRightDrive.setPower(backRightSpeedF);
        }

    } /*
    /*public Command periodic() {
        return infinite(()->{
        telemetry.addData("FrontLeftPower", frontLeftSpeedR);
        telemetry.addData("FrontRightPower", frontRightSpeedR);
        telemetry.addData("BackLeftPower", backLeftSpeedR);
        telemetry.addData("BackRightPower", backRightSpeedR);
        });
    }*/
}
