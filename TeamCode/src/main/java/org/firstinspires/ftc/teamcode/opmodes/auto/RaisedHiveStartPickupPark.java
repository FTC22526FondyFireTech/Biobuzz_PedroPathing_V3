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

import org.firstinspires.ftc.teamcode.commands.AlignToPoseCommand;
import org.firstinspires.ftc.teamcode.commands.CheckIfOdometryZoneTagsSeen;
import org.firstinspires.ftc.teamcode.utils.PedroPaths;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NectarShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PollenShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsysytem;
import org.firstinspires.ftc.teamcode.utils.AutoCommands;
import org.firstinspires.ftc.teamcode.utils.Configurables;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.FieldConstants;
import org.firstinspires.ftc.teamcode.utils.GetRPMFromDistance;
import org.firstinspires.ftc.teamcode.utils.GlobalData;

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
@Autonomous(name = "Raised Hive Start Basic", group = "Auto")
public class RaisedHiveStartPickupPark extends CommandOpMode {

    private MecanumDriveSubsystem drive;
    private IntakeSubsystem intake;
    private PollenShooterSubsystem pss;
    private NectarShooterSubsystem nss;
    private VisionSubsysytem vss;
    private Follower follower;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private Pose startPose, flowerApproachPose, controlPointPose, flowerPickupPose, parkPose, parkPoseFromStart;
    private Path approachFlower, pickupFlower, retractFlower, returnToStart, park, parkFromStart;

    private boolean isBlueAlliance;

    private ElapsedTime autoTimer;

    private AutoCommands ac;
    private GetRPMFromDistance grpmpollenfd;
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
        parkPose = forAlliance(FieldConstants.redAudienceParkPose, isBlueAlliance);
        parkPoseFromStart = forAlliance(FieldConstants.redAudienceParkPoseFromStart, isBlueAlliance);
        controlPointPose = forAlliance(FieldConstants.redAllianceParkfromStartControlPointPose, isBlueAlliance);


        // Only this line differs from PedroAutoSample.initialize() - everything below is
        // identical Follower/Path/FollowPathCommand usage.


        follower = Constants.createFollower(this.hardwareMap);

        follower.setPose(startPose);

        buildPaths();
        hiveHasFallen = false;

        schedule(
                ShootPickupPark()
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


    private Command ShootPickupPark() {
        return Commands.sequence(
                pss.setTargetRPMCommand(PollenShooterSubsystem.autoShootRPM),

                shoot(),

                new CheckIfOdometryZoneTagsSeen(vss, false).withTimeout(5000),

                pickup(),

                Commands.defer(() -> new ConditionalCommand(
                                park(),
                                Commands.sequence(
                                        moveForMissedShoot(),
                                        shoot(),
                                        parkFromStart()),

                                () -> !vss.getZoneTagsSeen()),

                        Collections.emptyList()));
    }


    public Command pickup() {
        return Commands.sequence(
                Commands.parallel(
                        checkSequenceStepCommand(),
                        new FollowPathCommand(follower, approachFlower),
                        intake.runIntakeCommand(),
                        intake.armDownCommand()),
                checkSequenceStepCommand(),
                new WaitCommand(1000),
                new FollowPathCommand(follower, pickupFlower).setGlobalMaxPower(0.25),
                new WaitCommand(1500),
                new FollowPathCommand(follower, retractFlower),
                new WaitCommand(250),
                checkSequenceStepCommand());
    }

    public Command moveForMissedShoot() {

        return Commands.sequence(
                new FollowPathCommand(follower, returnToStart, true),
                checkSequenceStepCommand(),
                Commands.parallel(
                        intake.stopIntakeCommand(),
                        intake.armUpCommand()),
                Commands.waitMillis(500),    Commands.runOnce(() -> follower.stop()),
                Commands.defer(() -> new AlignToPoseCommand(drive).asProxy(),
                        Collections.emptyList()));
    }

    private Command shoot() {
        return ac.autoShootPollenCommand();
    }


    public Command park() {
        return new FollowPathCommand(follower, park, false);
    }

//    public Command parkAfterSecondShoot() {
//        return new FollowPathCommand(follower, parkFromStart, false).setGlobalMaxPower(1.);
//    }

    public Command parkAfterSecondShoot() {
        return Commands.sequence(
                new FollowPathCommand(follower, approachFlower).setGlobalMaxPower(1.),
                Commands.waitMillis(250),
                new FollowPathCommand(follower, park));
    }

    public Command parkFromStart() {
        return new FollowPathCommand(follower, parkFromStart, false).setGlobalMaxPower(1.);
    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();
        autoTimer.reset();

        while (!isStopRequested() && opModeIsActive() && GlobalData.allianceIsConfirmed && autoTimer.seconds() <= 30) {
            run();

            follower.update();


            distancetoTarget = getDistanceToTarget(GlobalData.getActiveHiveTargetPose(), drive.getPose());

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

        approachFlower = buildBezierLine(startPose, flowerApproachPose);
        pickupFlower = buildBezierLine(flowerApproachPose, flowerPickupPose);
        retractFlower = buildBezierLine(flowerPickupPose, flowerApproachPose);
        returnToStart = buildBezierLine(flowerApproachPose, startPose);
        park = buildBezierLine(flowerPickupPose, parkPose);
        parkFromStart = buildBezierCurve(startPose, controlPointPose, parkPoseFromStart);
    }

    Path buildBezierLine(Pose start, Pose end) {
        return PedroPaths.line(start, end);
    }

    Path buildBezierCurve(Pose start, Pose control, Pose end) {
        return PedroPaths.curveTangent(start, control, end);
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
