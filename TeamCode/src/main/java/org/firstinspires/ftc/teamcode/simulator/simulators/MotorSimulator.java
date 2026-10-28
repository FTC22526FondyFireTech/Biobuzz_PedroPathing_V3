package org.firstinspires.ftc.teamcode.simulator.simulators;

/**
 * Simulates a single drive motor with realistic physics.
 * <p>
 * Takes power input ([-1, 1]) and generates:
 * - Simulated motor velocity (RPM)
 * - Encoder tick output
 * - Current draw estimation
 * <p>
 * Accounts for:
 * - Motor inertia and acceleration curves
 * - Friction and deceleration
 * - Encoder resolution and wheel diameter
 */
public class MotorSimulator {

    // Motor characteristics (tuned for REV Robotics motors)
    private static final double MOTOR_INERTIA = 0.002; // kg·m²
    private static final double ACCELERATION_TIME = 0.150; // seconds to reach max power

    // Encoder configuration
    private static final double ENCODER_TICKS_PER_REV = 288; // Core Hex 1.1
    private static final double WHEEL_DIAMETER_INCHES = 3.78; // goBILDA mecanum wheel

    private static final double WHEEL_INCHES_PER_REV = Math.PI * WHEEL_DIAMETER_INCHES;//3.14*3.78 = 11.7
    private static final double TICKS_PER_INCH = ENCODER_TICKS_PER_REV / WHEEL_INCHES_PER_REV;//288/11.7 = 24

    //  private static final double MAX_IPM = MAX_RPM * WHEEL_INCHES_PER_REV;//435 * 11.7 = 5000

    private double max_RPM; // REV Core Hex 1.1
    // Motor state
    private double currentVelocityRPM = 0;
    private double currentPower = 0;
    private long totalEncoderTicks = 0;

    private boolean inverted;

    public MotorSimulator(boolean inverted,double maxRPM) {
        this.currentVelocityRPM = 0;
        this.totalEncoderTicks = 0;
        this.max_RPM = maxRPM;
        this.inverted=inverted;
    }

    /**
     * Update motor simulation for one time step.
     *
     * @param targetPower [-1.0, 1.0] motor power command
     * @param dt          time step in seconds
     */
    public void update(double targetPower, double dt) {

        currentPower = targetPower;

        if (isInverted()) currentPower = -targetPower;

        // Target velocity based on power
        double targetVelocityRPM = targetPower * max_RPM;

        // Simulate acceleration/deceleration with time constant
        // Exponential approach: v(t) = v_target + (v_current - v_target) * e^(-t/tau)
        double tau = ACCELERATION_TIME / 2.2; // time constant (empirical)
        double acceleration = (targetVelocityRPM - currentVelocityRPM) / tau * dt;
        currentVelocityRPM += acceleration;

        // Clamp to physical limits
        currentVelocityRPM = Math.max(-max_RPM, Math.min(max_RPM, currentVelocityRPM));

        // Calculate encoder ticks this frame
        // RPM -> ticks per second -> ticks this frame
        double ticksPerSecond = (currentVelocityRPM / 60.0) * ENCODER_TICKS_PER_REV;
        long ticksThisFrame = Math.round(ticksPerSecond * dt);
        totalEncoderTicks += ticksThisFrame;
    }

    /**
     * Set motor power directly (useful for test mode).
     */
    public void setPower(double power) {
        currentPower = Math.max(-1.0, Math.min(1.0, power));
    }

    /**
     * Get current motor velocity in RPM.
     */
    public double getVelocityRPM() {
        return currentVelocityRPM;
    }

    public double getVelocityIPM() {
        return getVelocityRPM() * WHEEL_INCHES_PER_REV;
    }

    /**
     * Get total encoder ticks accumulated since start.
     */
    public long getEncoderTicks() {
        return totalEncoderTicks;
    }

    /**
     * Get linear distance traveled (inches).
     */
    public double getDistanceInches() {
        return totalEncoderTicks / TICKS_PER_INCH;
    }

    /**
     * Get this simulator's tick-to-inch conversion factor, so consumers
     * (e.g. OdometrySimulator) decode ticks at the same resolution they were generated.
     */
    public static double getTicksPerInch() {
        return TICKS_PER_INCH;
    }

    /**
     * Reset encoder to zero.
     */
    public void resetEncoder() {
        totalEncoderTicks = 0;
    }

    /**
     * Get current power command [-1, 1].
     */
    public double getPower() {
        return currentPower;
    }

    public boolean isInverted() {
        return inverted;
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }
}
