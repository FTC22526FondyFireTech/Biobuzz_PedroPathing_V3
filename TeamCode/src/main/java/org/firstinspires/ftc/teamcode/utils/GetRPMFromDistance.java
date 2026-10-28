package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.simulator.drivetrains.MecanumDriveSubsystemSimulation;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NectarShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PollenShooterSubsystem;

public class GetRPMFromDistance {

    PollenShooterSubsystem pss;

    NectarShooterSubsystem nss;
    MecanumDriveSubsystemSimulation driveSim;

    MecanumDriveSubsystem drive;

    RollingAverage pollenRa = new RollingAverage(10);
    RollingAverage nectarRa = new RollingAverage(10);

    public GetRPMFromDistance(MecanumDriveSubsystemSimulation driveSim, PollenShooterSubsystem pss) {
        this.pss = pss;
        this.driveSim = driveSim;
    }

    public GetRPMFromDistance(MecanumDriveSubsystem drive, PollenShooterSubsystem pss) {
        this.pss = pss;
        this.drive = drive;
    }

    public GetRPMFromDistance(MecanumDriveSubsystem drive, NectarShooterSubsystem nss) {
        this.nss = nss;
        this.drive = drive;
    }


    public double getPollenRPMFromDistanceSim(double inches) {
        double distToTarget = getDistanceToTarget(GlobalData.getActiveHiveTargetPose(), driveSim.getOdometry().getRobotPose());
        double targetRPMFromDistance = pss.getTargetRPMFromDistance(distToTarget);
        pollenRa.add(targetRPMFromDistance);
        return pss.getTargetRPMFromDistance(distToTarget);

    }

    public double getPollenRPMFromDistance(double inches) {
        double distToTarget = getDistanceToTarget(GlobalData.getActiveHiveTargetPose(), drive.getPose());
        double targetRPMFromDistance = pss.getTargetRPMFromDistance(distToTarget);
        pollenRa.add(targetRPMFromDistance);
        return pss.getTargetRPMFromDistance(distToTarget);

    }
    public double getNectarRPMFromDistance(double inches) {
        double distToTarget = getDistanceToTarget(GlobalData.getActiveHiveTargetPose(), drive.getPose());
        double targetRPMFromDistance = pss.getTargetRPMFromDistance(distToTarget);
        nectarRa.add(targetRPMFromDistance);
        return nss.getTargetRPMFromDistance(distToTarget);
    }



    public double getDistanceToTarget(Pose targetPose, Pose robotPose) {
        double XDiff = targetPose.x() - robotPose.x();
        double YDiff = targetPose.y() - robotPose.y();
        return Math.hypot(XDiff, YDiff);
    }

}