package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsysytem;
import org.firstinspires.ftc.teamcode.utils.GlobalData;
import org.firstinspires.ftc.teamcode.utils.HiveMotionTracker;

import java.util.Collections;
import java.util.List;

public class CheckIfOdometryZoneTagsSeen extends CommandBase {
    private VisionSubsysytem vss;
    private boolean lookForHiveUp;

    private boolean lookForHiveDown;
    private HiveMotionTracker hmt;

    private int tagsZone;

    List<Integer> tagsSeen = Collections.emptyList();
    double Y;
    private final double yHiveRisenValue = 0;
    private boolean zoneTagsSeen;
    List<LLResultTypes.FiducialResult> fr = Collections.emptyList();

    public CheckIfOdometryZoneTagsSeen(VisionSubsysytem vss, boolean lookForHiveUp) {
        this.vss = vss;
        this.lookForHiveUp = lookForHiveUp;
    }

    @Override
    public void initialize() {
        lookForHiveDown = !lookForHiveUp;
        hmt = new HiveMotionTracker();
    }


    @Override
    public void execute() {

        LLResult result = vss.getLLResults();

        if (result.isValid()) {
            hmt.update(result);

            vss.hiveFalling = hmt.getMotion() == HiveMotionTracker.Motion.FALLING;
            vss.hiveRising = hmt.getMotion() == HiveMotionTracker.Motion.RISING;
            vss.hiveStill = hmt.getMotion() == HiveMotionTracker.Motion.STILL;
            vss.latestHiveTy = hmt.getLatestTy();
            vss.hiveRateDegPerSec = hmt.getRateDegPerSec();
        }


        fr = vss.getFiducialResults();
        if (!fr.isEmpty()) {
            tagsSeen = vss.getTagsSeen(fr);
        }

        tagsZone = vss.findZoneFromTags(tagsSeen);
        Y = vss.getGroupTargetPose(fr)[1];
        vss.setInOdometryZone(tagsZone == GlobalData.getCurrentOdometryZone());
        zoneTagsSeen = vss.getZoneTagsSeen();

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return (!fr.isEmpty() && zoneTagsSeen && lookForHiveUp)
                || (!fr.isEmpty() && !zoneTagsSeen && lookForHiveDown);
    }

    public void showTelemetry(TelemetryManager telemetryM){
        telemetryM.addData("No Fiducial Results",fr.isEmpty());
        telemetryM.addData("Zone", tagsZone);
        telemetryM.addData("Y", Y);
        telemetryM.addData("Tags Seen", tagsSeen);
        telemetryM.addData("Hive Rising", vss.hiveRising);
        telemetryM.addData("Hive Falling", vss.hiveFalling);
        telemetryM.addData("Hive Still", vss.hiveStill);
    }


}
