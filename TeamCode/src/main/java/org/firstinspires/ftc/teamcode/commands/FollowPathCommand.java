package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * Pedro Pathing 3 replacement for SolversLib's Pedro 2 {@code FollowPathCommand}, with the same
 * constructor shapes. Starts the {@link Path} on the {@link Follower} and finishes once the
 * follower is no longer following it. Takes the Follower directly and declares no subsystem
 * requirement, like the SolversLib version it replaces.
 * <p>
 * {@code holdEnd == false} lets the follower stop driving at the end of the path instead of
 * holding the end pose, matching Pedro 2's {@code followPath(path, holdEnd)}.
 * <p>
 * {@link #setGlobalMaxPower} scales the path via Foresight's {@code maxPathSpeed} for this path
 * only. Check on the robot that this behaves like Pedro 2's global max power once Foresight is
 * tuned.
 */
public class FollowPathCommand extends CommandBase {

    private final Follower follower;
    private final Path path;
    private final boolean holdEnd;
    private Double maxPower = null;

    public FollowPathCommand(Follower follower, Path path) {
        this(follower, path, true);
    }

    public FollowPathCommand(Follower follower, Path path, boolean holdEnd) {
        this.follower = follower;
        this.path = path;
        this.holdEnd = holdEnd;
    }

    public FollowPathCommand setGlobalMaxPower(double power) {
        this.maxPower = power;
        return this;
    }

    @Override
    public void initialize() {
        follower.holdEnd.set(holdEnd);
        Path toFollow = path;
        if (maxPower != null) {
            toFollow = path.with(Constants.foresightConfig.maxPathSpeed.at(maxPower));
        }
        follower.follow(toFollow);
    }

    @Override
    public boolean isFinished() {
        return !follower.following();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) follower.stop();
    }
}
