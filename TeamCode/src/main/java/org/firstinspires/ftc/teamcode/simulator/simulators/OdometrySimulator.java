package org.firstinspires.ftc.teamcode.simulator.simulators;

import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;

/**
 * Simulates mecanum-wheel + gyro localization (no dead-wheel odometry pods).
 *
 * Translation (forward/strafe) is recovered from the four drive wheel encoders
 * via mecanum forward kinematics - the exact inverse of the drivetrain's power
 * mixing (FL=f+r+w, FR=f-r-w, BL=f-r+w, BR=f+r-w). Heading comes from a
 * {@link GyroSimulator} instead of wheel differential, matching how a real
 * mecanum robot without dead wheels localizes: wheel encoders for translation,
 * IMU for heading (immune to wheel-slip-induced drift).
 */
public class OdometrySimulator {

    // Motor simulators for each wheel
    private final MotorSimulator frontLeftMotor;
    private final MotorSimulator frontRightMotor;
    private final MotorSimulator backLeftMotor;
    private final MotorSimulator backRightMotor;
    private final GyroSimulator gyro;

    // Tick-to-inch conversion: derived from MotorSimulator's tick resolution so decoded
    // distances match what MotorSimulator actually generates.
    private static final double TICKS_TO_INCHES = 1.0 / MotorSimulator.getTicksPerInch();//1/24 = .004

    // Robot pose
    private double poseX = 0.0;      // inches
    private double poseY = 0.0;      // inches
    private double poseHeading = 0.0; // radians

    // Previous encoder readings (for calculating deltas)
    private long prevLeftFrontTicks = 0;
    private long prevRightFrontTicks = 0;
    private long prevLeftRearTicks = 0;
    private long prevRightRearTicks = 0;

    // Debug output
    private double lastDeltaForward = 0;
    private double lastDeltaStrafe = 0;
    private double lastDeltaHeading = 0;

    public OdometrySimulator(MotorSimulator frontLeft, MotorSimulator frontRight,
                            MotorSimulator backLeft, MotorSimulator backRight, GyroSimulator gyro) {
        this.frontLeftMotor = frontLeft;
        this.frontRightMotor = frontRight;
        this.backLeftMotor = backLeft;
        this.backRightMotor = backRight;
        this.gyro = gyro;
    }

    /**
     * Update odometry pose based on current encoder + gyro values.
     * Call this every control loop iteration, after the gyro has been updated.
     */
    public void update() {
        // Get current encoder readings from motors
        long leftFrontTicks = frontLeftMotor.getEncoderTicks();
        long rightFrontTicks = frontRightMotor.getEncoderTicks();
        long leftRearTicks = backLeftMotor.getEncoderTicks();
        long rightRearTicks = backRightMotor.getEncoderTicks();

        // Calculate deltas since last update
        long deltaFL = leftFrontTicks - prevLeftFrontTicks;
        long deltaFR = rightFrontTicks - prevRightFrontTicks;
        long deltaBL = leftRearTicks - prevLeftRearTicks;
        long deltaBR = rightRearTicks - prevRightRearTicks;

        // Convert ticks to inches (per-wheel: each mecanum wheel's motion is a mix
        // of forward + strafe + rotate, unlike a dedicated odometry pod)
        double fl = deltaFL * TICKS_TO_INCHES;
        double fr = deltaFR * TICKS_TO_INCHES;
        double bl = deltaBL * TICKS_TO_INCHES;
        double br = deltaBR * TICKS_TO_INCHES;

        // Mecanum forward kinematics: inverse of drive()'s
        // FL=f+r+w, FR=f-r-w, BL=f-r+w, BR=f+r-w mixing.
        // Using all four wheels exactly recovers forward/strafe independent of
        // the robot's other motion components (a single wheel's ticks cannot,
        // since every wheel mixes all three commands together).
        double avgForward = (fl + fr + bl + br) / 4.0;
        double strafe = (fl - fr - bl + br) / 4.0;

        // Heading comes from the gyro, not wheel differential: wheel-based heading
        // drifts under mecanum wheel slip, while a gyro measures actual rotation directly.
        double newHeading = gyro.getHeadingRadians();
        double headingDelta = newHeading - poseHeading;
        poseHeading = newHeading;

        // Convert movement vectors to global frame.
        // `strafe` here is physical strafe-RIGHT (matches drive()'s `right` parameter).
        // Pedro's Vector/Pose math uses the standard CCW convention throughout (heading
        // increases CCW, and the rotation matrix below assumes local +Y is 90 degrees CCW
        // from local +X/forward - i.e. LEFT, not right). So the local Y-component fed into
        // the rotation must be -strafe, or a right-strafe command ends up displacing the
        // robot's global pose to the left instead of the right, which the Follower's PID
        // then "corrects" in the wrong direction - a runaway positive-feedback drift.
        // Rotate by the average heading over this update to get global coordinates
        double cos_heading = Math.cos(poseHeading - headingDelta / 2.0);
        double sin_heading = Math.sin(poseHeading - headingDelta / 2.0);

        // Global displacement
        double globalDX = avgForward * cos_heading + strafe * sin_heading;
        double globalDY = avgForward * sin_heading - strafe * cos_heading;

        // Update pose
        poseX += globalDX;
        poseY += globalDY;

        // Store for next iteration
        prevLeftFrontTicks = leftFrontTicks;
        prevRightFrontTicks = rightFrontTicks;
        prevLeftRearTicks = leftRearTicks;
        prevRightRearTicks = rightRearTicks;

        // Store for debugging
        lastDeltaForward = avgForward;
        lastDeltaStrafe = strafe;
        lastDeltaHeading = headingDelta;
    }

    /**
     * Reset pose to origin.
     */
    public void resetPose() {
        poseX = 0.0;
        poseY = 0.0;
        poseHeading = gyro.getHeadingRadians();

        // Reset encoder references
        prevLeftFrontTicks = frontLeftMotor.getEncoderTicks();
        prevRightFrontTicks = frontRightMotor.getEncoderTicks();
        prevLeftRearTicks = backLeftMotor.getEncoderTicks();
        prevRightRearTicks = backRightMotor.getEncoderTicks();
    }

    public Command resetPoseCommand() {
        return new InstantCommand(this::resetPose);
    }


    /**
     * Set pose to specific values (e.g., after AprilTag correction).
     */
    public void setPose(double x, double y, double heading) {
        poseX = x;
        poseY = y;
        poseHeading = heading;
        gyro.setHeadingDegrees(Math.toDegrees(heading));
    }

    public Command setPoseCommand(Pose pose) {
        return new InstantCommand(() -> setPose(pose.x(),pose.y(),pose.heading()));
    }


    // ---- Getters for telemetry ----

    public double getX() {
        return poseX;
    }

    public double getY() {
        return poseY;
    }

    public double getHeading() {
        return poseHeading;
    }

    public Pose getRobotPose(){
        return new Pose(getX(),getY(),getHeading());
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(poseHeading);
    }

    /**
     * Get individual encoder values (useful for debugging).
     */
    public long getLeftFrontTicks() {
        return frontLeftMotor.getEncoderTicks();
    }

    public long getRightFrontTicks() {
        return frontRightMotor.getEncoderTicks();
    }

    public long getLeftRearTicks() {
        return backLeftMotor.getEncoderTicks();
    }

    public long getRightRearTicks() {
        return backRightMotor.getEncoderTicks();
    }

    public double getGyroAngularVelocityDegPerSec() {
        return gyro.getAngularVelocityDegPerSec();
    }

    /**
     * Debug info for last update.
     */
    public double getLastDeltaForward() {
        return lastDeltaForward;
    }

    public double getLastDeltaStrafe() {
        return lastDeltaStrafe;
    }

    public double getLastDeltaHeading() {
        return lastDeltaHeading;
    }
}
