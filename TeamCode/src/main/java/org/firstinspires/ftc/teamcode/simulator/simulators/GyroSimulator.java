package org.firstinspires.ftc.teamcode.simulator.simulators;

/**
 * Simulates a robot-mounted IMU/gyro heading sensor.
 *
 * Unlike wheel-based heading (which drifts under mecanum wheel slip and
 * cross-coupling), a real gyro measures the robot's actual angular velocity
 * directly. This is fed from the commanded rotate power (not from wheel
 * ticks), mirroring how a real gyro's reading is independent of drivetrain
 * encoders. It integrates using the same acceleration-curve model as
 * {@link MotorSimulator} so turning responds with comparable physical inertia
 * instead of snapping instantly to full rate.
 */
public class GyroSimulator {

    // Tune to match the real robot's measured max turn rate.
    private static final double MAX_ANGULAR_VELOCITY_DEG_PER_SEC = 270;
    private static final double ACCELERATION_TIME = 0.150; // seconds to reach max turn rate

    private double currentAngularVelocityDegPerSec = 0;
    private double headingDeg = 0;

    /**
     * Update gyro simulation for one time step.
     *
     * @param targetRotatePower [-1.0, 1.0] commanded rotate power (same signal fed into
     *                          the drivetrain's mecanum mixing, before it's combined with
     *                          forward/strafe)
     * @param dt time step in seconds
     */
    public void update(double targetRotatePower, double dt) {
        double targetAngularVelocity = targetRotatePower * MAX_ANGULAR_VELOCITY_DEG_PER_SEC;

        double tau = ACCELERATION_TIME / 2.2; // time constant (empirical, matches MotorSimulator)
        double acceleration = (targetAngularVelocity - currentAngularVelocityDegPerSec) / tau * dt;
        currentAngularVelocityDegPerSec += acceleration;

        currentAngularVelocityDegPerSec = Math.max(-MAX_ANGULAR_VELOCITY_DEG_PER_SEC,
                Math.min(MAX_ANGULAR_VELOCITY_DEG_PER_SEC, currentAngularVelocityDegPerSec));

        headingDeg += currentAngularVelocityDegPerSec * dt;
    }

    public double getHeadingRadians() {
        return Math.toRadians(headingDeg);
    }

    public double getHeadingDegrees() {
        return headingDeg;
    }

    public double getAngularVelocityDegPerSec() {
        return currentAngularVelocityDegPerSec;
    }

    /**
     * Reset accumulated heading to zero (e.g. at the start of an OpMode).
     */
    public void reset() {
        headingDeg = 0;
        currentAngularVelocityDegPerSec = 0;
    }

    /**
     * Overwrite the current heading (e.g. after an AprilTag/vision correction),
     * without disturbing the angular velocity being tracked.
     */
    public void setHeadingDegrees(double degrees) {
        headingDeg = degrees;
    }
}
