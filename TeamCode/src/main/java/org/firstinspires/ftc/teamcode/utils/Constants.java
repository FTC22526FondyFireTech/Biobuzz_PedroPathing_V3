package org.firstinspires.ftc.teamcode.utils;

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

/**
 * Central tuning/config file for the mecanum drivebase (Pedro Pathing 3).
 * <p>
 * This wires the goBILDA Pinpoint odometry computer and four drive motors into a
 * Pedro Pathing {@link Follower}. The Follower does all of the localization math and
 * path following; {@code MecanumDriveSubsystem} just wraps it in a SolversLib
 * {@code SubsystemBase} so it plays nicely with the command-based framework.
 * <p>
 * TODO: The Pedro 2 PIDF / zero-power-acceleration / velocity values do not carry over to
 * Pedro 3. Run the Foresight tuner in AutoTune (http://192.168.43.1:10158 while connected to
 * the Robot Controller) and paste its generated Java into {@link #foresightConfig}. The
 * Mecanum and Pinpoint tuners likewise generate {@link #drivetrainConfig} and
 * {@link #localizerConfig}; the values below were carried over from the Pedro 2 setup.
 */
public class Constants {

    // ---- Hardware map device names (must match your Driver Station robot config) ----
    public static final String FRONT_LEFT_MOTOR_NAME = "frontLeft";
    public static final String FRONT_RIGHT_MOTOR_NAME = "frontRight";
    public static final String BACK_LEFT_MOTOR_NAME = "backLeft";
    public static final String BACK_RIGHT_MOTOR_NAME = "backRight";

    public static final String PINPOINT_NAME = "pinpoint";

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(FRONT_LEFT_MOTOR_NAME);
        c.backLeftName.set(BACK_LEFT_MOTOR_NAME);
        c.frontRightName.set(FRONT_RIGHT_MOTOR_NAME);
        c.backRightName.set(BACK_RIGHT_MOTOR_NAME);
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // goBILDA Pinpoint odometry computer configuration.
    // xPodOffset is the forward pod's offset and yPodOffset the strafe pod's offset from the
    // robot's center of rotation (Pedro 2's forwardPodY / strafePodX), in inches.
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(PINPOINT_NAME);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(6.5);
        c.yPodOffset.set(-5.0);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    });

    // PLACEHOLDER values estimated from the Pedro 2 tuning so the follower can start (the
    // required fields below have no library default; leaving any unset throws "config variable
    // has not been set"). They are NOT tuned for Foresight: replace this whole block with the
    // Foresight AutoTune output before relying on path following on the robot.
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        // Pedro 2 translational kP: primary 0.075, secondary (near the target) 0.4
        Controller primaryTranslationalForward = Controller.proportional(0.075);
        Controller secondaryTranslationalForward = Controller.proportional(0.4);
        Controller primaryTranslationalLateral = Controller.proportional(0.075);
        Controller secondaryTranslationalLateral = Controller.proportional(0.4);

        c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

        // ~1 / max velocity
        c.coast.set(Controller.proportionalFeedforward(0.0124));
        c.brake.set(Controller.proportionalFeedforward(0.0124));

        // Pedro 2 heading kP 1.1
        c.headingFeedback.set(Controller.proportional(1.1));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0, 0.01));

        // Braking distance ~ v^2 / (2 * natural deceleration)
        c.linearBrakeCoefficients.set(Matrix.diag(0.0, 0.0));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.0117, 0.0095));

        // Pedro 2 xVelocity / yVelocity and |zero-power accelerations| (in/s, in/s^2)
        c.maxAchievableForwardVelocity.set(80.73);
        c.maxAchievableStrafeVelocity.set(58.54);
        c.naturalForwardDeceleration.set(42.92);
        c.naturalStrafeDeceleration.set(52.68);
    });

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig));
    }
}

/*
 * Pedro 2 values retained for reference while re-tuning with Foresight:
 *   mass 7.17 kg, forwardZeroPowerAcceleration -42.92, lateralZeroPowerAcceleration -52.68,
 *   xVelocity 80.73 in/s, yVelocity 58.54 in/s
 *   Three-wheel pods (unused): forward/strafe/turn ticks->inches ~0.0020073, leftPodY 6.25,
 *   rightPodY -6.25, strafePodX -6.75, encoders frontLeft / backRight / backLeft(strafe reversed)
 */
