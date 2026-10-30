package org.firstinspires.ftc.teamcode.simulator.commands;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.simulator.drivetrains.MecanumDriveSubsystemSimulation;
import org.firstinspires.ftc.teamcode.utils.GlobalData;

import java.util.function.DoubleSupplier;

/**
 * Lets the driver translate freely (field-centric) while the robot automatically keeps its
 * front pointed at a fixed point on the field, using Pinpoint-based pose from Pedro.
 * <p>
 * Heading error is computed every loop from the live pose, so the aim stays locked even as the
 * robot moves around the target. The turn output comes from a PID on the wrapped heading error
 * and is fed to the subsystem as the turn input; the driver's rotation stick is ignored.
 * <p>
 * Usage (hold a button to aim):
 * <pre>{@code
 * new GamepadEx(gamepad1).getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
 *     .whileHeld(new AimAtPoseCommand(drive, new Pose(12, 132),
 *         () -> -gamepad1.left_stick_y, () -> -gamepad1.left_stick_x));
 * }</pre>
 * <p>
 * Target pose uses Pedro field coordinates (inches, 0-144). Only its x/y are used.
 * Requires the drive subsystem, so it interrupts the default DriveCommand while active and
 * the default command resumes when this command ends.
 */
public class AlignToPoseCommandSim extends CommandBase {

    // Live-tunable from the Panels Configurables tab.
    public static double kP = 2.5;
    public static double kI = 0.0;
    public static double kD = 0.0;
    /**
     * Heading error (radians) below which turn output is zeroed to avoid jitter.
     */
    public static double deadbandRad = Math.toRadians(2.0);
    /**
     * Limit on turn power so aiming never saturates the motors.
     */
    public static double maxTurn = .5;
    /**
     * Added to the computed angle, e.g. Math.PI if your shooter faces the back of the robot.
     */
    public static double headingOffsetRad = Math.PI;

    private final MecanumDriveSubsystemSimulation drive;
    private Pose target;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final PIDController headingController = new PIDController(kP, kI, kD);

    TelemetryManager telemetryM;
    private double error;

    private int test;

    public AlignToPoseCommandSim(MecanumDriveSubsystemSimulation drive, DoubleSupplier forward, DoubleSupplier strafe) {
        this.drive = drive;
        this.forward = forward;
        this.strafe = strafe;
        addRequirements(drive);
    }

    public AlignToPoseCommandSim(MecanumDriveSubsystemSimulation drive) {
        this.drive = drive;
        this.forward = () -> 0;
        this.strafe = () -> 0;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        headingController.reset();
        headingController.setPID(kP, kI, kD);
      //  drive.startTeleopDrive();
//        telemetryM.addData("INIT", "AimAtPoseCommand");
//        telemetryM.update();
    }

    @Override
    public void execute() {

        test++;
        telemetryM.addData("TEST", test);

        target = GlobalData.getActiveHiveTargetPose();


        Pose pose = drive.getOdometry().getRobotPose();

        double desiredHeading = Math.atan2(target.y() - pose.y(), target.x() - pose.x())
                + headingOffsetRad;


        error = (wrapAngle(desiredHeading - pose.heading()));

        double turn = 0;

        if (Math.abs(error) > deadbandRad) {
            // Setpoint = error, measurement = 0, so the PID sees exactly the wrapped error.
            turn = headingController.calculate(error, 0);
            turn = Math.max(-maxTurn, Math.min(maxTurn, turn));
        }

        double distToTarget = getDistanceToTarget(target, pose);

//        telemetryM.addData("Aim Dist2Tgt,in", distToTarget);
//        telemetryM.addData("AimDesiredHeading", Math.toDegrees(desiredHeading));
//        telemetryM.addData("Aim Turn", turn);
//        telemetryM.addData("Aim Target X", target.x());
//        telemetryM.addData("Aim Target Y", target.y());
//        telemetryM.addData("FWD", forward.getAsDouble());
//        telemetryM.addData("STRFE", strafe.getAsDouble());
//        telemetryM.addData("Aim TurnErr", Math.toDegrees(error));
//        drive.showTelemetry(telemetryM);
        // Pedro: positive turn = counterclockwise, matching positive heading error here.
        drive.drive(forward.getAsDouble(), strafe.getAsDouble(), turn);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(error) <= deadbandRad; // runs while held / until interrupted
    }

    /**
     * Wraps an angle to (-PI, PI] so the robot always turns the short way.
     */
    private static double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }


    public double getDistanceToTarget(Pose targetPose, Pose robotPose) {
        double XDiff = targetPose.x() - robotPose.x();
        double YDiff = targetPose.y() - robotPose.y();
        return Math.hypot(XDiff, YDiff);
    }

}
