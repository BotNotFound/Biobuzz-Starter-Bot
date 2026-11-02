package org.firstinspires.ftc.teamcode.pedro;

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
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FL");
        c.frontRightName.set("FR");
        c.backLeftName.set("BL");
        c.backRightName.set("BR");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.24844464354627716);
        c.yPodOffset.set(-2.7867625078817055);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.20341749380399432);
                Controller secondaryTranslationalForward = Controller.proportional(0.07515732854503597);
                Controller primaryTranslationalLateral = Controller.proportional(0.2648542805391562);
                Controller secondaryTranslationalLateral = Controller.proportional(0.09785657962250255);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.016980333392648404));
                c.brake.set(Controller.proportionalFeedforward(0.014433283383751142));

                c.headingFeedback.set(Controller.proportional(4.725310077581414));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04833417601164856, 0.005247118273232745));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07928137116605187, 0.04640218309819594));
                c.quadraticBrakeCoefficients.set(Matrix.diag(9.292618152372629E-4, 0.0015041073038427712));

                c.maxAchievableForwardVelocity.set(61.655695479444944);
                c.maxAchievableStrafeVelocity.set(51.333380208996594);
                c.naturalForwardDeceleration.set(38.13638359729856);
                c.naturalStrafeDeceleration.set(58.20285479663184);
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