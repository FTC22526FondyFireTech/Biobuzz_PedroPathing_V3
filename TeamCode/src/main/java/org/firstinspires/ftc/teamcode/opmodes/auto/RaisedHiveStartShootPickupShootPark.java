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
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.AlignToPoseCommand;
import org.firstinspires.ftc.teamcode.commands.CheckIfOdometryZoneTagsSeen;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NectarShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PollenShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsysytem;
import org.firstinspires.ftc.teamcode.utils.AutoCommands;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.FieldConstants;
import org.firstinspires.ftc.teamcode.utils.GlobalData;
import org.firstinspires.ftc.teamcode.utils.PedroPaths;

import java.util.Collections;

/*
 * {@link MecanumDriveSubsystemSimulation} instead of real hardware - no robot required.
 * Swap {@link Constants#createFollower(com.qualcomm.robotcore.hardware.HardwareMap)}
 * for {@link SimulatorConstants#createSimulatedFollower(MecanumDriveSubsystemSimulation)} and
 * everything else (Paths, FollowPathCommand, follower.update()/getPose()) works unchanged,
 * since both factories hand back a real {@code Follower}.
 */

/**
 * Used only when partner can score in the initial raised hive.
 * Robot starts at the red or blue low hive and waits until it raises before shooting.
 * Then moves to pick up from adjacent flower and shoots those before parking.
 * If hive doesn't raise before 20 seconds robot goes straight to park.
 *
 *
 *
 *
 */
@Autonomous(name = "Raised Hive Start Extended", group = "Auto")
public class RaisedHiveStartShootPickupShootPark extends CommandOpMode {
    private MecanumDriveSubsystem drive;
    private IntakeSubsystem intake;
    private PollenShooterSubsystem pss;
    private NectarShooterSubsystem nss;
    private VisionSubsysytem vss;
    private Follower follower;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private Pose startPose, flowerApproachPose, controlPointPose, flowerPickupPose, parkPose, parkPoseFromStart, secondShootPose, controlPointPoseSecondShoot;
    private Path approachFlowerPath, pickupFlowerPath, retractFlowerPath, returnToStart, parkPath, parkFromStartPath,
            moveToSecondShootPath, parkAfterSecondShootPath;

    private boolean isBlueAlliance;

    private ElapsedTime autoTimer;
    private AutoCommands ac;

    private int seqStep;
    private double distancetoTarget;
    private double targetRPMFromDistance;

    public boolean hiveHasFallen = true;

    @Override
    public void initialize() {
        super.reset();
        isBlueAlliance = GlobalData.isBlueAlliance();


        drive = new MecanumDriveSubsystem(this.hardwareMap);

        intake = new IntakeSubsystem(this.hardwareMap);

        pss = new PollenShooterSubsystem(this.hardwareMap);

        nss = new NectarShooterSubsystem(this.hardwareMap);

        vss = new VisionSubsysytem(this);

        ac = new AutoCommands(drive, pss, nss);

        GlobalData.autoHasRun = true;

        autoTimer = new ElapsedTime();
        startPose = forAlliance(FieldConstants.redAudienceHiHiveStartPose, isBlueAlliance);
        flowerApproachPose = forAlliance(FieldConstants.redAllianceWallFlowerApproachPose, isBlueAlliance);
        flowerPickupPose = forAlliance(FieldConstants.redAllianceWallFlowerPickupPose, isBlueAlliance);
        parkPose = forAlliance(FieldConstants.redScoringParkPose, GlobalData.isBlueAlliance());
        parkPoseFromStart = forAlliance(FieldConstants.redAudienceParkPoseFromStart, isBlueAlliance);

        secondShootPose = forAlliance(FieldConstants.redScoringSecondShootPose, isBlueAlliance);
        controlPointPose = forAlliance(FieldConstants.redAllianceParkfromStartControlPointPose, isBlueAlliance);
        controlPointPoseSecondShoot = forAlliance(FieldConstants.redSecondMoveToShootControlPointPose, isBlueAlliance);


        // Only this line differs from PedroAutoSample.initialize() - everything below is
        // identical Follower/Path/FollowPathCommand usage.

        follower = Constants.createFollower(this.hardwareMap);

        follower.setPose(startPose);

        buildPaths();
        hiveHasFallen = false;

        schedule(
                ShootPickupShootPark()
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


    private Command ShootPickupShootPark() {
        return Commands.sequence(

                pss.setTargetRPMCommand(PollenShooterSubsystem.autoShootRPM),
                shoot(),
                new CheckIfOdometryZoneTagsSeen(vss, false).withTimeout(5000),

                pickup(),
                Commands.defer(() -> new ConditionalCommand(
                                Commands.sequence(
                                        moveForSecondShoot(),
                                        Commands.runOnce(() -> follower.stop()),
                                        new AlignToPoseCommand(drive),
                                        shoot(),
                                        parkFromSecondShoot()),
                                Commands.sequence(
                                        moveForMissedShoot(),
                                        Commands.runOnce(() -> follower.stop()),
                                        new AlignToPoseCommand(drive),
                                        shoot(),
                                        parkFromStart()),

                                () -> !vss.getZoneTagsSeen()),

                        Collections.emptyList()));
    }


    public Command pickup() {
        return Commands.sequence(
                Commands.parallel(
                        checkSequenceStepCommand(),
                        new FollowPathCommand(follower, approachFlowerPath),
                        intake.runIntakeCommand(),
                        intake.armDownCommand()),
                checkSequenceStepCommand(),
                new WaitCommand(1000),
                new FollowPathCommand(follower, pickupFlowerPath).setGlobalMaxPower(0.25),
                new WaitCommand(1500),
                checkSequenceStepCommand(),
                new FollowPathCommand(follower, retractFlowerPath),
                new WaitCommand(250),
                checkSequenceStepCommand(),
                Commands.parallel(
                        intake.stopIntakeCommand(),
                        intake.armUpCommand()));
    }

    public Command moveForMissedShoot() {

        return Commands.sequence(
                new FollowPathCommand(follower, returnToStart, true),
                checkSequenceStepCommand(),
                Commands.parallel(
                        intake.stopIntakeCommand(),
                        intake.armUpCommand()),
                Commands.waitMillis(500),
                // Release Pedro's end-pose hold so it doesn't override the align turn.
                Commands.runOnce(() -> follower.stop()),
                new AlignToPoseCommand(drive));
    }

    public Command moveForSecondShoot() {
        return new FollowPathCommand(follower, moveToSecondShootPath, true);
    }

    private Command shoot() {
        return ac.autoShootPollenCommand();
    }


    public Command parkFromStart() {
        return new FollowPathCommand(follower, parkFromStartPath, false);
    }

    public Command parkFromSecondShoot() {
        return new FollowPathCommand(follower, parkAfterSecondShootPath, false);
    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();
        autoTimer.reset();

        while (!isStopRequested() && opModeIsActive() && GlobalData.allianceIsConfirmed && autoTimer.seconds() <= 30) {
            run();

            follower.update();

//            distancetoTarget = getDistanceToTarget(GlobalData.getActiveHiveTargetPose(), driveSim.getOdometry().getRobotPose());
//            telemetryM.addData("RC", driveSim.isRobotCentric());
//            targetRPMFromDistance = grpmpollenfd.getPollenRPMFromDistanceSim(distancetoTarget);


            telemetryM.addData("ENDPOSX", parkFromStartPath.endPose().x());
            telemetryM.addData("ENDPOSX=Y", parkFromStartPath.endPose().y());
            telemetryM.addData("ENDPOSH", parkFromStartPath.endPose().heading());
            telemetryM.addData("ROBPOSX", drive.getPose().x());
            telemetryM.addData("ROBPOSY", drive.getPose().y());
            telemetryM.addData("ROBPOSH", drive.getPose().heading());
            telemetryM.addData("DistRem", follower.remainingDistance());
            telemetryM.addData("PathCompl", follower.completion());


            telemetryM.addData("Auto Timer", autoTimer.seconds());
            telemetryM.addData("Follower Is Busy", follower.isBusy());
            telemetryM.addData("Shooting", pss.shooterServoTargetPosition == PollenShooterSubsystem.shootPosition);
            telemetryM.addData("Intake Running", intake.intakeRunning);
            telemetryM.addData("Intake Arm Down", intake.getArmPosition() == IntakeSubsystem.downPosition);

            telemetryM.addData("SeqStep", seqStep);

            telemetryM.update(telemetry);
        }
        reset();
    }

    public void buildPaths() {

        approachFlowerPath = PedroPaths.line(startPose, flowerApproachPose);
        pickupFlowerPath = PedroPaths.line(flowerApproachPose, flowerPickupPose);
        retractFlowerPath = PedroPaths.line(flowerPickupPose, flowerApproachPose);
        returnToStart = PedroPaths.line(flowerApproachPose, startPose);
        parkFromStartPath = PedroPaths.curve(startPose, controlPointPose, parkPoseFromStart);
        moveToSecondShootPath = PedroPaths.curve(flowerApproachPose, controlPointPoseSecondShoot, secondShootPose);
        parkAfterSecondShootPath = PedroPaths.line(secondShootPose, parkPose);
    }


    public double getDistanceToTarget(Pose targetPose, Pose robotPose) {
        double XDiff = targetPose.x() - robotPose.x();
        double YDiff = targetPose.y() - robotPose.y();
        return Math.hypot(XDiff, YDiff);
    }

    public Command checkSequenceStepCommand() {
        return Commands.runOnce(() -> seqStep++);
    }


}
