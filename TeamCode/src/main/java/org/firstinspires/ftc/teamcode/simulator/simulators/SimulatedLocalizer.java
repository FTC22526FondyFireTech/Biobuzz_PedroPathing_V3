package org.firstinspires.ftc.teamcode.simulator.simulators;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.simulator.drivetrains.MecanumDriveSubsystemSimulation;

/**
 * Feeds a Pedro Pathing 3 {@code Follower} from {@link OdometrySimulator} instead of real
 * hardware.
 *
 * OdometrySimulator's pose is kept current every physics tick by
 * {@link MecanumDriveSubsystemSimulation#periodic()},
 * independent of how often {@code Follower.update()} (and therefore this class's
 * {@link #update()}) gets called. So {@link #update()} only needs to derive a velocity
 * estimate from the pose delta since the last call.
 */
public class SimulatedLocalizer implements Localizer {

    private final OdometrySimulator odometry;

    private Pose lastPose;
    private long lastNanos = System.nanoTime();
    private Velocity currentVelocity = Velocity.zero();

    public SimulatedLocalizer(OdometrySimulator odometry) {
        this.odometry = odometry;
        this.lastPose = odometry.getRobotPose();
    }

    @Override
    public void setPose(Pose pose) {
        odometry.setPose(pose.x(), pose.y(), pose.heading());
        lastPose = pose;
        currentVelocity = Velocity.zero();
    }

    @Override
    public MotionState state() {
        return MotionState.ofVelocity(odometry.getRobotPose(), currentVelocity);
    }

    @Override
    public void update() {
        long now = System.nanoTime();
        double dt = (now - lastNanos) * 1e-9;
        lastNanos = now;

        Pose pose = odometry.getRobotPose();
        if (dt > 0) {
            double dx = pose.x() - lastPose.x();
            double dy = pose.y() - lastPose.y();
            double dh = pose.heading() - lastPose.heading();
            // keep heading delta in [-pi, pi] so a wrap-around doesn't spike velocity
            while (dh > Math.PI) dh -= 2 * Math.PI;
            while (dh < -Math.PI) dh += 2 * Math.PI;

            currentVelocity = new Velocity(dx / dt, dy / dt, dh / dt);
        }
        lastPose = pose;
    }

    @Override
    public void reset() {
        currentVelocity = Velocity.zero();
        lastPose = odometry.getRobotPose();
    }
}
