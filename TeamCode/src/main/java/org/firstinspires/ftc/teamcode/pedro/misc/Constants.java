package org.firstinspires.ftc.teamcode.pedro.misc;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(
        c -> {
            c.frontLeftName.set("LF");
            c.frontRightName.set("RF");
            c.backLeftName.set("LB");
            c.backRightName.set("RB");
            c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
            c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
        }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(
        c -> {
            c.name.set("pinpoint");
            c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            c.xPodOffset.set(2.9603501567690396);
            c.yPodOffset.set(2.3553128880778638);
            c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
            c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.offsetUnits.set(DistanceUnit.INCH);
        }
    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
        c -> {
            Controller primaryTranslationalForward = Controller.proportional(0.187066467726583);
            Controller secondaryTranslationalForward = Controller.proportional(0.06911606131689593);
            Controller primaryTranslationalLateral = Controller.proportional(0.4718775540637554);
            Controller secondaryTranslationalLateral = Controller.proportional(0.17434614742609342);

            c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
            c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

            c.coast.set(Controller.proportionalFeedforward(0.013622296701368252));
            c.brake.set(Controller.proportionalFeedforward(0.011578952196163015));

            c.headingFeedback.set(Controller.proportional(3.783319268657998));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05458376016951765, 0.0032813085586550343));

            c.linearBrakeCoefficients.set(Matrix.diag(0.08137759773980223, 0.056908298031753124));
            c.quadraticBrakeCoefficients.set(Matrix.diag(0.0010074116940598724, 0.0013572088073360257));

            c.maxAchievableForwardVelocity.set(74.16659320382855);
            c.maxAchievableStrafeVelocity.set(65.54197229339107);
            c.naturalForwardDeceleration.set(42.10401427322364);
            c.naturalStrafeDeceleration.set(46.47399526265213);
        }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}