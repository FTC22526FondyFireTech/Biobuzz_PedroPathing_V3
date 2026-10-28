package org.firstinspires.ftc.teamcode.simulator.commands;

import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.simulator.drivetrains.MecanumDriveSubsystemSimulation;
import org.firstinspires.ftc.teamcode.subsystems.NectarShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PollenShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsysytem;
import org.firstinspires.ftc.teamcode.utils.GlobalData;

public class AutoCommandsSim {

    private final MecanumDriveSubsystemSimulation driveSim;

    private final PollenShooterSubsystem pss;
    private final NectarShooterSubsystem nss;

    public AutoCommandsSim(MecanumDriveSubsystemSimulation driveSim, PollenShooterSubsystem pss, NectarShooterSubsystem nss) {
        this.driveSim = driveSim;
        this.pss = pss;
        this.nss = nss;

    }

    public Command autoShootPollenCommand() {

        return Commands.sequence(
                pss.runPollenShooterAtVelocityCommand(),
                new WaitUntilCommand(pss::pollenShooterAtSpeed),
                pss.shootPollenCommand(),
                new WaitCommand(2000),
                pss.holdPollenCommand(),
                pss.stopPollenShooterCommand());
    }



    public double getDistanceToTarget(Pose targetPose, Pose robotPose) {
        double XDiff = targetPose.x() - robotPose.x();
        double YDiff = targetPose.y() - robotPose.y();
        return Math.sqrt(Math.pow(XDiff, 2) - Math.pow(YDiff, 2));
    }

    public double getDistanceFromOdometry() {
        return getDistanceToTarget(GlobalData.getActiveHiveTargetPose(), driveSim.getOdometry().getRobotPose());
    }


    public Command pollenShootUsingDistanceCommand() {

        return Commands.sequence(
                pss.runPollenShooterAtVelocityCommand(),
                new WaitUntilCommand(pss::pollenShooterAtSpeed),
                pss.shootPollenCommand(),
                new WaitCommand(2000),
                pss.holdPollenCommand(),
                pss.stopPollenShooterCommand());
    }

    public Command nectarShootUsingDistanceCommand() {
        return Commands.sequence(
                nss.setTargetRPMCommand(nss.getTargetRPMFromDistance(getDistanceFromOdometry())),
                nss.runNectarShooterAtVelocityCommand(),
                new WaitUntilCommand(nss::nectarShooterAtSpeed),
                nss.shootNectarCommand(),
                new WaitCommand(2000),
                nss.holdNectarCommand(),
                nss.stopNectarShooterCommand());
    }

}

