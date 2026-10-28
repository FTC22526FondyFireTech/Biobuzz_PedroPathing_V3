package org.firstinspires.ftc.teamcode.simulator.drivetrains;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;

import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Drives a {@link MecanumDriveSubsystemSimulation} from a Pedro Pathing 3 {@code Follower}.
 *
 * Pedro's real {@code com.pedropathing.revhub.drivetrains.Mecanum} can't be reused directly
 * here - its constructor requires a {@code HardwareMap} and real motors. The wheel-power mixing
 * ({@link #computeWheelPowersUnnormalized}), the normalization in {@link #drive}, the headroom
 * calculation in {@link #maxScaling} and {@link #interpolateVelocity} are the same math as that
 * class (pure math, no hardware dependency), kept in lockstep so this produces the same wheel
 * powers for the same inputs. Wheel order is [FL, FR, BL, BR], which is also the order
 * {@link MecanumDriveSubsystemSimulation#setDrivePowers} expects.
 */
public class SimulatedMecanumDrivetrainForFollower implements Drivetrain {

    private final MecanumDriveSubsystemSimulation driveSim;
    private final double[] wheelPowers = new double[4];

    public SimulatedMecanumDrivetrainForFollower(MecanumDriveSubsystemSimulation driveSim) {
        this.driveSim = driveSim;
    }

    public double[] computeWheelPowersUnnormalized(DrivePowers p) {
        double f = p.forward(), s = p.strafe(), t = p.turn();
        return new double[]{
                f - s - t, // FL
                f + s + t, // FR
                f + s - t, // BL
                f - s + t  // BR
        };
    }

    @Override
    public void drive(DrivePowers drivePowers, boolean brake) {
        double[] raw = computeWheelPowersUnnormalized(drivePowers);
        double max = 1.0;
        for (double w : raw) max = Math.max(max, Math.abs(w));
        for (int i = 0; i < 4; i++) wheelPowers[i] = raw[i] / max;
        driveSim.setDrivePowers(wheelPowers.clone());
    }

    /**
     * Largest scale in [0, 1] that can be applied to {@code add} on top of {@code base}
     * before any wheel saturates.
     */
    @Override
    public double maxScaling(DrivePowers base, DrivePowers add) {
        double scale = 1.0;
        double[] b = computeWheelPowersUnnormalized(base);
        double[] a = computeWheelPowersUnnormalized(add);
        for (int i = 0; i < 4; i++) {
            if (Math.abs(a[i]) < 1e-9) continue;
            double hi = (1.0 - b[i]) / a[i];
            double lo = (-1.0 - b[i]) / a[i];
            if (hi >= 0 && hi < scale) scale = hi;
            if (lo >= 0 && lo < scale) scale = lo;
        }
        return Math.max(0.0, Math.min(1.0, scale));
    }

    @Override
    public void stop() {
        stop(true);
    }

    @Override
    public void stop(boolean brake) {
        Arrays.fill(wheelPowers, 0);
        driveSim.setDrivePowers(new double[]{0, 0, 0, 0});
    }

    @Override
    public Map<String, Object> debug() {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("wheelPowers", Arrays.toString(wheelPowers));
        return m;
    }

    @Override
    public double interpolateVelocity(double forwardVelocity, double strafeVelocity, double angle) {
        return 1.0 / (Math.abs(Math.cos(angle)) / forwardVelocity
                + Math.abs(Math.sin(angle)) / strafeVelocity);
    }
}
