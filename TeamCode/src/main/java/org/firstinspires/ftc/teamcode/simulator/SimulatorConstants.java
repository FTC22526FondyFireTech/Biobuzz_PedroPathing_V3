package org.firstinspires.ftc.teamcode.simulator;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.simulator.drivetrains.MecanumDriveSubsystemSimulation;
import org.firstinspires.ftc.teamcode.simulator.drivetrains.SimulatedMecanumDrivetrainForFollower;
import org.firstinspires.ftc.teamcode.simulator.simulators.SimulatedLocalizer;
import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * Builds a Follower that drives {@link MecanumDriveSubsystemSimulation} instead of real
 * hardware, for running the same Paths/autonomous code against the simulator.
 * {@code Follower} itself has no hardware dependency - it only needs a {@code Localizer},
 * a {@code Drivetrain} and an {@code Algorithm} - so this constructs the real
 * {@link Follower} directly, reusing {@link Constants#foresightConfig} so simulated runs
 * follow paths the same way as the real robot.
 */
public class SimulatorConstants {

    public static Follower createSimulatedFollower(MecanumDriveSubsystemSimulation driveSim) {
        SimulatedLocalizer localizer = new SimulatedLocalizer(driveSim.getOdometry());
        SimulatedMecanumDrivetrainForFollower drivetrain = new SimulatedMecanumDrivetrainForFollower(driveSim);
        return new Follower(localizer, drivetrain, new Foresight(Constants.foresightConfig));
    }


    public static final double length = 144;
    public static final double width = 144;

}
