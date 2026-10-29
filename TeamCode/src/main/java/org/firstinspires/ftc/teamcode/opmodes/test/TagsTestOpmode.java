package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.commands.CheckIfOdometryZoneTagsSeen;
import org.firstinspires.ftc.teamcode.simulator.drivetrains.MecanumDriveSubsystemSimulation;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsysytem;
import org.firstinspires.ftc.teamcode.utils.GlobalData;
import org.firstinspires.ftc.teamcode.utils.HiveMotionTracker;

import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "TagsTest")
@Disabled

public class TagsTestOpmode extends CommandOpMode {

    TelemetryManager telemetryM;
    VisionSubsysytem vss;
    Timing.Timer readCamera;
    long readMilliSecs = 250;
    int HIVE_TAG_ID = 31;

    MecanumDriveSubsystemSimulation driveSim;
    HiveMotionTracker hmt;

    GamepadEx driverGamepad;

    @Override
    public void initialize() {
        driverGamepad = new GamepadEx(gamepad1);
        vss = new VisionSubsysytem(this);
        hmt = new HiveMotionTracker();

        driveSim = new MecanumDriveSubsystemSimulation(this);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryM.update(telemetry);
        readCamera = new Timing.Timer(readMilliSecs, TimeUnit.MILLISECONDS);
        readCamera.start();

        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new CheckIfOdometryZoneTagsSeen(vss,true));
    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            run();
            showTelemetry();
            telemetryM.update(telemetry);

        }
        reset();
    }

    public double getAngleDegreesToTarget(Pose targetPose, Pose robotPose) {
        double XDiff = targetPose.x() - robotPose.x();
        double YDiff = targetPose.y() - robotPose.y();
        return Math.toDegrees(Math.atan2(YDiff, XDiff));
    }

    public double getDistanceToTarget(Pose targetPose, Pose robotPose) {
        double XDiff = targetPose.x() - robotPose.x();
        double YDiff = targetPose.y() - robotPose.y();
        return Math.sqrt(Math.pow(XDiff, 2) - Math.pow(YDiff, 2));
    }

    public void showTelemetry() {

        //  if (readCamera.done()) {

        double dist = 0;

        dist = getDistanceToTarget(GlobalData.getActiveHiveTargetPose(), driveSim.getOdometry().getRobotPose());

        telemetryM.addData("Distance(from odometry)", f3(dist));

        List<LLResultTypes.FiducialResult> fr = vss.getFiducialResults();

        List<Integer> tagsSeen = vss.getTagsSeen(fr);
        telemetryM.addData("TagsSeen", tagsSeen);


        telemetryM.addData("ZoneFromTags", vss.findZoneFromTags(tagsSeen));
        telemetryM.addData("Current Zone", vss.getCurrentTagZone());
        telemetryM.addData("Is In Current Zone", vss.isInCurrentZone(fr));

        if (!fr.isEmpty()) {
            double[] temp = vss.getGroupTargetPose(fr);
            double x = temp[0];
            double y = temp[1];
            double z = temp[2];
            double yaw = temp[3];
            double pitch = temp[4];
            telemetryM.addData("Distance(from tags)", f0M2I(z));


            telemetryM.addData("X", f3M2I(x));
            telemetryM.addData("Y", f3M2I(y));
            telemetryM.addData("Z", f3M2I(z));
            telemetryM.addData("Yaw", f3(yaw));
            telemetryM.addData("Pitch", f3(pitch));


        }

        LLResult result = vss.getLLResults();

        if (result.isValid()) {

            //   telemetryM.addData("tx", f3(result.getTx()));
            telemetryM.addData("ty", f3(result.getTy()));
            telemetryM.addData("ta", f3(result.getTa()));
            telemetryM.addData("Tag Count", vss.getNumberTagsSeen(fr));


            hmt.update(result);
            telemetryM.addData("Hive Falling", hmt.getMotion()==HiveMotionTracker.Motion.FALLING);
            telemetryM.addData("ty (deg)", f3(hmt.getLatestTy()));
            telemetryM.addData("rate (deg/s)", f3(hmt.getRateDegPerSec()));
            telemetryM.addData("samples", hmt.getSampleCount());

        }


    }

    static String f3(double v) {
        return String.format(Locale.US, "%.3f", v);
    }

    static String f3M2I(double v) {
        return String.format(Locale.US, "%.3f", v * 39.37);
    }

    static String f0M2I(double v) {
        return String.format(Locale.US, "%.0f", v * 39.37);
    }

}


