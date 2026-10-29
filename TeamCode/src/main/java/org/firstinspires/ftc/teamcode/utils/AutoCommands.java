package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NectarShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PollenShooterSubsystem;

public class AutoCommands {

    private final MecanumDriveSubsystem drive;


    private final PollenShooterSubsystem pss;
    private final NectarShooterSubsystem nss;

    private final double startShootRPM = 1500;


    public AutoCommands(MecanumDriveSubsystem drive, PollenShooterSubsystem pss, NectarShooterSubsystem nss) {
        this.drive = drive;
        this.pss = pss;
        this.nss = nss;
    }

    public Command autoShootPollenCommand() {
        return Commands.sequence(
                pss.setTargetRPMCommand(startShootRPM),
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
        return getDistanceToTarget(GlobalData.getActiveHiveTargetPose(), drive.getPose());
    }


    public Command pollenShootUsingDistanceCommand(double distance) {

        return Commands.sequence(
                pss.runPollenShooterAtVelocityCommand(),
                new WaitUntilCommand(pss::pollenShooterAtSpeed),
                pss.shootPollenCommand(),
                new WaitCommand(2000),
                pss.holdPollenCommand(),
                pss.stopPollenShooterCommand());
    }

    public Command nectarShootUsingDistanceCommand() {
        double distance = getDistanceFromOdometry();
        nss.setTargetRPM(nss.getTargetRPMFromDistance(distance));
        return Commands.sequence(
                nss.runNectarShooterAtVelocityCommand(),
                new WaitUntilCommand(nss::nectarShooterAtSpeed),
                nss.shootNectarCommand(),
                new WaitCommand(2000),
                nss.holdNectarCommand(),
                nss.stopNectarShooterCommand());
    }

}

