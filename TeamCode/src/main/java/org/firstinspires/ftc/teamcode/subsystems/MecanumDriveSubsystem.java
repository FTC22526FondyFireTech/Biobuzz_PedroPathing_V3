package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.Drawing;
import org.firstinspires.ftc.teamcode.utils.FieldConstants;
import org.firstinspires.ftc.teamcode.utils.GlobalData;

/**
 * Mecanum drivebase subsystem.
 * <p>
 * Hardware/localization is delegated entirely to a Pedro Pathing {@link Follower}, which
 * owns the four drive motors and the goBILDA Pinpoint odometry computer (wired up in
 * {@link Constants}). This class is the SolversLib-facing wrapper: it registers with the
 * {@code CommandScheduler} as a normal {@code SubsystemBase}, drives the Follower's update
 * loop from {@link #periodic()}, pushes live pose/path data to the FTC Panels dashboard,
 * and exposes simple drive/path-following methods for Commands to call.
 * <p>
 * Only one Command should control this subsystem at a time; the scheduler enforces that
 * automatically as long as commands declare it with {@code addRequirements(driveSubsystem)}.
 */
public class MecanumDriveSubsystem extends SubsystemBase {

    private final Follower follower;
    private boolean teleopDriveActive = false;
    public boolean isRobotCentric = false;

    public boolean isRobotCentric() {
        return isRobotCentric;
    }

    private ElapsedTime et;

    public void setRobotCentric(boolean robotCentric) {
        isRobotCentric = robotCentric;
    }

    public MecanumDriveSubsystem(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
        Drawing.init();
        et = new ElapsedTime();
    }

    public MecanumDriveSubsystem(HardwareMap hardwareMap, Pose startingPose) {
        this(hardwareMap);
        follower.setPose(startingPose);
    }

    /**
     * Runs once per scheduler loop (SubsystemBase auto-registers this). Advances the
     * Follower's localization + path-following state machine and pushes telemetry to
     * both the Driver Station and the FTC Panels dashboard.
     */
    @Override
    public void periodic() {
        follower.update();
        if (et.seconds() > .5) {
            setActiveHiveTargetPose();
            et.reset();
        }
        if (GlobalData.autoHasRun) GlobalData.robotPose = getPose();

        Drawing.drawRobot(getPose(), GlobalData.robotLook);
        Drawing.drawLine(getPose().x(),getPose().y(),
                GlobalData.getActiveHiveTargetPose().x(),GlobalData.getActiveHiveTargetPose().y());
        Drawing.sendPacket();
    }

    public void setActiveHiveTargetPose() {

        double x = follower.pose().x();
        double y = follower.pose().y();

        if (x < 72 && y >= 72) {
            GlobalData.setCurrentOdometryZone(0);
            GlobalData.setActiveHiveTargetPose(FieldConstants.redScoringHiveTargetPose);
            GlobalData.setActiveStartPose(FieldConstants.redScoringLoHiveStartPose);
        } else if (x < 72 && y < 72) {
            GlobalData.setCurrentOdometryZone(1);
            GlobalData.setActiveHiveTargetPose(FieldConstants.redAudienceHiveTargetPose);
            GlobalData.setActiveStartPose(FieldConstants.redAudienceHiHiveStartPose);
        } else if (x >= 72 && y < 72) {
            GlobalData.setCurrentOdometryZone(2);
            GlobalData.setActiveHiveTargetPose(FieldConstants.toBlue(FieldConstants.redAudienceHiveTargetPose));
            GlobalData.setActiveStartPose(FieldConstants.toBlue(FieldConstants.redScoringLoHiveStartPose));
        } else if (x >= 72 && y > 72) {
            GlobalData.setCurrentOdometryZone(3);
            GlobalData.setActiveHiveTargetPose(FieldConstants.toBlue(FieldConstants.redScoringHiveTargetPose));
            GlobalData.setActiveStartPose(FieldConstants.toBlue(FieldConstants.redAudienceHiHiveStartPose));
        }

    }

    public void showTelemetry(TelemetryManager telemetryM) {
        telemetryM.debug("Drive X (in)", follower.pose().x());
        telemetryM.debug("Drive Y (in)", follower.pose().y());
        telemetryM.debug("Drive Heading (deg)", Math.toDegrees(follower.pose().heading()));
        telemetryM.debug("Drive Busy", follower.following());
    }

    /**
     * Switches the Follower into open, driver-controlled mode. Call this once before
     * feeding it stick inputs (a default teleop DriveCommand should do this in its
     * initialize()). Automatically re-armed by {@link #driveRobotCentric} /
     * {@link #driveFieldCentric} if a path-following command just finished.
     */
    public void startTeleopDrive() {
        follower.manual(0, 0, 0);
        teleopDriveActive = true;
    }

    /**
     * Drives with stick input relative to the robot's own heading.
     */
    public void driveRobotCentric(double forward, double strafe, double turn) {
        if (!teleopDriveActive) startTeleopDrive();
        follower.manual(forward, strafe, turn);
    }

    /**
     * Drives with stick input relative to the field (forward is always "away from driver").
     */
    public void driveFieldCentric(double forward, double strafe, double turn) {
        if (!teleopDriveActive) startTeleopDrive();
        follower.manual(ManualDrive.fieldCentric(forward, strafe, turn, follower.pose().heading()));
    }

    public void drive(double forward, double strafe, double turn) {
        if (!teleopDriveActive) startTeleopDrive();
        if (isRobotCentric) follower.manual(forward, strafe, turn);
        else follower.manual(ManualDrive.fieldCentric(forward, strafe, turn, follower.pose().heading()));
    }

    public void stop() {
        driveFieldCentric(0, 0, 0);
    }

    /**
     * Follows an autonomous path and holds its end heading/position once finished.
     */
    public void followPath(Path path) {
        followPath(path, true);
    }

    public void followPath(Path path, boolean holdEnd) {
        teleopDriveActive = false;
        follower.holdEnd.set(holdEnd);
        follower.follow(path);
    }

    /**
     * True while the Follower is actively executing a path.
     */
    public boolean isBusy() {
        return follower.following();
    }

    public Pose getPose() {
        return follower.pose();
    }

    /**
     * Overwrites the Follower's pose estimate, e.g. after an AprilTag correction.
     */
    public void setPose(Pose pose) {
        follower.setPose(pose);
    }


    public Command setPoseCommand(Pose pose) {
        return new InstantCommand(() -> setPose(pose));
    }

    public Command resetPoseCommand() {
        return new InstantCommand(() -> setPose(new Pose(0, 0, 0)));
    }

    /**
     * Escape hatch for direct access to the Pedro 3 Follower (hold, manual, pose history) outside this class.
     */
    public Follower getFollower() {
        return follower;
    }
}

/*  top down robot view:


                      front
             -------------------------
     ^
   + |         left pod      right pod
              (port 0)        (port 3)

   - |
     v        strafe pod
                (port 2)

             -------------------------
                      back

                    < - Y - >
                      +   -


    x length: 17 in
    y length: 17.25 in
    center: (8.5, 8.625) (x, y)
 */
