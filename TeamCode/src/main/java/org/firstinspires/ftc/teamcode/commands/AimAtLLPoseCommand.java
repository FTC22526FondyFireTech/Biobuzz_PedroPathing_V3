package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsysytem;
import org.firstinspires.ftc.teamcode.utils.GlobalData;

import java.util.List;
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
@Configurable
public class AimAtLLPoseCommand extends CommandBase {

    // Live-tunable from the Panels Configurables tab.
    public static double headingkP = .5;
    public static double headingkI = 0.0;
    public static double headingkD = 0.0;

    public static double strafekP = .5;
    public static double strafekI = 0.0;
    public static double strafekD = 0.0;

    public static boolean setPID = false;
    /**
     * Heading error (radians) below which turn output is zeroed to avoid jitter.
     */
    public static double deadbandRad = Math.toRadians(1.0);
    /**
     * Limit on turn power so aiming never saturates the motors.
     */
    public static double maxTurn = .5;
    public static double maxStrafe = .5;
    /**
     * Added to the computed angle, e.g. Math.PI if your shooter faces the back of the robot.
     */
    public static double headingOffsetRad = 0.0;

    private final MecanumDriveSubsystem drive;
    private final VisionSubsysytem vss;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final PIDController headingController = new PIDController(headingkP, headingkI, headingkD);
    private final PIDController strafeController = new PIDController(strafekP, strafekI, strafekI);
    TelemetryManager telemetryM;

    public AimAtLLPoseCommand(MecanumDriveSubsystem drive, VisionSubsysytem vss,
                              DoubleSupplier forward, DoubleSupplier strafe) {
        this.drive = drive;
        this.vss = vss;
        this.forward = forward;
        this.strafe = strafe;
        addRequirements(drive);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void initialize() {
        headingController.reset();
        headingController.setPID(headingkP, headingkI, headingkD);
        strafeController.setPID(strafekP, strafekI, strafekD);
        drive.startTeleopDrive();
    }

    @Override
    public void execute() {

        if (setPID) {
            headingController.setPID(headingkP, headingkI, headingkD);
            strafeController.setPID(strafekP, strafekI, strafekD);
        }
        List<LLResultTypes.FiducialResult> fr = vss.getFiducialResults();

        double[] temp = vss.getGroupTargetPose(fr);
        double x = temp[0];
        double y = temp[1];
        double z = temp[2];
        double yaw = temp[3];
        double pitch = temp[4];

        double strafe = strafeController.calculate(x, 0);
        double turn = headingController.calculate(yaw, 0);
        turn = Math.max(-maxTurn, Math.min(maxTurn, turn));
        strafe = Math.max(-maxStrafe, Math.min(maxStrafe, strafe));

        if (GlobalData.isRedAlliance()) {
            drive.drive(forward.getAsDouble(), strafe, turn);
        } else {
            drive.drive(-forward.getAsDouble(), -strafe, turn);
        }

       // drive.drive(forward.getAsDouble(), strafe, turn);

    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }


    @Override
    public boolean isFinished() {
        return false; // runs while held / until interrupted
    }


    /**
     * Wraps an angle to (-PI, PI] so the robot always turns the short way.
     */
    private static double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
