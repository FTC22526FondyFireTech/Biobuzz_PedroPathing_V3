package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.Configurables;
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
//@Configurable
public class AimAtPoseCommand extends CommandBase {

    // Live-tunable from the Panels Configurables tab.
    public static double kP = .5;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static boolean setPID = false;
    /**
     * Heading error (radians) below which turn output is zeroed to avoid jitter.
     */
    public static double deadbandRad = Math.toRadians(1.0);
    /**
     * Limit on turn power so aiming never saturates the motors.
     */
    public static double maxTurn = .5;
    /**
     * Added to the computed angle, e.g. Math.PI if your shooter faces the back of the robot.
     */
    public static double headingOffsetRad = 0.0;

    private final MecanumDriveSubsystem drive;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final PIDController headingController = new PIDController(kP, kI, kD);
    TelemetryManager telemetryM;

    public AimAtPoseCommand(MecanumDriveSubsystem drive,
                            DoubleSupplier forward, DoubleSupplier strafe) {
        this.drive = drive;

        this.forward = forward;
        this.strafe = strafe;
        addRequirements(drive);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void initialize() {

        headingController.reset();
        headingController.setPID(kP, kI, kD);
    }

    @Override
    public void execute() {


        headingController.setPID(kP, kI, kD);

        Pose target = GlobalData.getActiveHiveTargetPose();


        Pose pose = drive.getPose();
        double desiredHeading = Math.atan2(target.y() - pose.y(), target.x() - pose.x())
                + headingOffsetRad;
        double error = wrapAngle(desiredHeading - pose.heading());

        double turn = 0;
        if (Math.abs(error) > deadbandRad) {
            // Setpoint = error, measurement = 0, so the PID sees exactly the wrapped error.
            turn = headingController.calculate(error, 0);
            turn = Math.max(-maxTurn, Math.min(maxTurn, turn));
            turn = -turn;
        }
//
        if (GlobalData.isRedAlliance()) {
            drive.drive(forward.getAsDouble(), strafe.getAsDouble(), turn);
        } else {
            drive.drive(-forward.getAsDouble(), -strafe.getAsDouble(), turn);
        }
        // Pedro: positive turn = counterclockwise, matching positive heading error here.
        //drive.drive(forward.getAsDouble(), strafe.getAsDouble(), turn);
//
//
        telemetryM.addData("AIPTurn", turn);
        telemetryM.addData("AIPTgfX", target.x());
        telemetryM.addData("AIPTurnErr", error);
        telemetryM.addData("AIPDESHDG", Math.toDegrees(desiredHeading));

    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    //
    @Override
    public boolean isFinished() {
        return false; // runs while held / until interrupted
    }
//

    /**
     * Wraps an angle to (-PI, PI] so the robot always turns the short way.
     */
    private static double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
