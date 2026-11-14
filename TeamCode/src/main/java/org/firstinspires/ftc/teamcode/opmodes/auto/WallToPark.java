package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.AutoCommands;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.FieldConstants;
import org.firstinspires.ftc.teamcode.utils.GlobalData;
import org.firstinspires.ftc.teamcode.utils.PedroPaths;

@Autonomous(name = "WallToPark", group = "Autonomous")
public class WallToPark extends CommandOpMode {

    private Follower follower;
    private MecanumDriveSubsystem drive;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private boolean isBlueAlliance;
    private ElapsedTime autoTimer;
    //private AutoCommands ac;
    public boolean hiveHasFallen = true;
    private Pose startPose, parkPose;

    private Path park;

    @Override
    public void initialize() {
        super.reset();
        isBlueAlliance = GlobalData.isBlueAlliance();

        drive = new MecanumDriveSubsystem(this.hardwareMap);
        //ac = new AutoCommands(drive, pss, nss);

        GlobalData.autoHasRun = true;

        autoTimer = new ElapsedTime();
        startPose = forAlliance(FieldConstants.redScoringCornerStartPose, isBlueAlliance);
        parkPose = forAlliance(FieldConstants.redScoringParkPose, isBlueAlliance);

        // Only this line differs from PedroAutoSample.initialize() - everything below is
        // identical Follower/Path/FollowPathCommand usage.

        follower = Constants.createFollower(this.hardwareMap);

        follower.setPose(startPose);

        buildPaths();
        hiveHasFallen = false;

        schedule(
                WallToParkZone()
        );
    }

    /**
     * BIOBUZZ: blue = red rotated 180° about field center (72,72).
     */

    static double FIELD = 144;

    public static Pose toBlue(Pose red) {
        return new Pose(
                FIELD - red.x(),
                FIELD - red.y(),
                AngleUnit.normalizeRadians(red.heading() + Math.PI));
    }

    public static Pose forAlliance(Pose redPose, boolean isBlue) {
        return isBlue ? toBlue(redPose) : redPose;
    }

    private Command WallToParkZone() {
        return new FollowPathCommand(follower, park, false);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();
        autoTimer.reset();

        while (!isStopRequested() && opModeIsActive() && GlobalData.allianceIsConfirmed && autoTimer.seconds() <= 30) {
            run();

            follower.update();

            telemetryM.addData("ROBPOSX", drive.getPose().x());
            telemetryM.addData("ROBPOSY", drive.getPose().y());
            telemetryM.addData("ROBPOSH", drive.getPose().heading());
            telemetryM.addData("DistRem", follower.remainingDistance());
            telemetryM.addData("PathCompl", follower.completion());


            telemetryM.addData("Auto Timer", autoTimer.seconds());
            telemetryM.addData("Follower Is Busy", follower.isBusy());

            telemetryM.update(telemetry);
        }
        reset();
    }

    public void buildPaths() {
        park = buildBezierLine(startPose, parkPose);
    }

    Path buildBezierLine(Pose start, Pose end) {
        return PedroPaths.line(start, end);
    }


    public double getDistanceToTarget(Pose targetPose, Pose robotPose) {
        double XDiff = targetPose.x() - robotPose.x();
        double YDiff = targetPose.y() - robotPose.y();
        return Math.hypot(XDiff, YDiff);
    }
}

