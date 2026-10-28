package org.firstinspires.ftc.teamcode.commands;
import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.simulator.drivetrains.MecanumDriveSubsystemSimulation;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

import java.util.concurrent.TimeUnit;

/**
 * Drives a mecanum robot to a target field pose (x, y, heading) using two PID loops:
 * one on straight-line distance to the target, one on heading.
 *
 * Conventions (match these to your localizer!):
 *   - Field coordinates in inches, +X forward, +Y left
 *   - Heading in radians, counter-clockwise positive
 *
 * The translation output is pointed straight at the target, so the robot drives
 * in a straight line while turning, rather than doing X and Y separately.
 */
public class DriveToPoseCommand extends CommandBase {

    // ---- Tuning constants: start small and tune on the robot ----
    public static double TRANSLATION_KP = 0.08, TRANSLATION_KI = 0.0, TRANSLATION_KD = 0.005;
    public static double HEADING_KP = 1.2, HEADING_KI = 0.0, HEADING_KD = 0.05;

    public static double MAX_DRIVE_POWER = 0.7;   // 0..1
    public static double MAX_TURN_POWER = 0.5;    // 0..1
    public static double MIN_DRIVE_POWER = 0.06;  // overcomes static friction near the target

    public static double POSITION_TOLERANCE_IN = 0.75;
    public static double HEADING_TOLERANCE_RAD = Math.toRadians(2.0);
    public static long SETTLE_TIME_MS = 500;       // must stay in tolerance this long to finish

    /**
     * SolversLib/FTCLib MecanumDrive treats positive turn as clockwise (turn right).
     * Our heading is CCW-positive, so the turn output is flipped. If the robot spins
     * the wrong way, set this to 1.0.
     */
    public static double TURN_SIGN = -1.0;

    private final MecanumDriveSubsystem drive;
    private final Pose target;
    private final PIDController translationPID =
            new PIDController(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD);
    private final PIDController headingPID =
            new PIDController(HEADING_KP, HEADING_KI, HEADING_KD);

    private final Timing.Timer settleTimer = new Timing.Timer(SETTLE_TIME_MS, TimeUnit.MILLISECONDS);
    private boolean inTolerance = false;

    public DriveToPoseCommand(MecanumDriveSubsystem drive, Pose target) {
        this.drive = drive;
        this.target = target;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.setRobotCentric(true);
        translationPID.reset();
        headingPID.reset();
        translationPID.setPID(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD);
        headingPID.setPID(HEADING_KP, HEADING_KI, HEADING_KD);
        inTolerance = false;
    }

    @Override
    public void execute() {
        Pose pose = drive.getPose();
        double robotHeading = pose.heading();

        // --- Errors in field frame ---
        double dx = target.x() - pose.x();
        double dy = target.y() - pose.y();
        double distance = Math.hypot(dx, dy);
        double headingError = angleWrap(target.heading() - robotHeading);

        // --- Translation: PID on distance, pointed along the error vector ---
        double drivePower = translationPID.calculate(0, distance); // error = distance
        drivePower = clamp(drivePower, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);
        if (distance > POSITION_TOLERANCE_IN && Math.abs(drivePower) < MIN_DRIVE_POWER) {
            drivePower = Math.copySign(MIN_DRIVE_POWER, drivePower);
        }

        double fieldVx = 0, fieldVy = 0;
        if (distance > 1e-6) {
            fieldVx = drivePower * dx / distance;
            fieldVy = drivePower * dy / distance;
        }

        // --- Rotate field-frame velocity into robot frame ---
        double cos = Math.cos(robotHeading), sin = Math.sin(robotHeading);
        double forward = fieldVx * cos + fieldVy * sin;
        double left    = -fieldVx * sin + fieldVy * cos;

        // --- Heading ---
        double turnPower = headingPID.calculate(0, headingError);
        turnPower = clamp(turnPower, -MAX_TURN_POWER, MAX_TURN_POWER);

        // MecanumDrive wants (strafe right, forward, turn clockwise)
        drive.drive( forward, -left,TURN_SIGN * turnPower);

        // --- Settle check ---
        boolean nowInTolerance = distance < POSITION_TOLERANCE_IN
                && Math.abs(headingError) < HEADING_TOLERANCE_RAD;
        if (nowInTolerance && !inTolerance) {
            settleTimer.start();
        }
        inTolerance = nowInTolerance;
    }

    @Override
    public boolean isFinished() {
        return inTolerance && settleTimer.done();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        drive.setRobotCentric(false);

    }

    private static double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
