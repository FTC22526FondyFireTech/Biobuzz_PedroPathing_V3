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
import org.firstinspires.ftc.teamcode.utils.PedroPaths;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.AlignToPoseCommand;
import org.firstinspires.ftc.teamcode.commands.CheckIfOdometryZoneTagsSeen;
import org.firstinspires.ftc.teamcode.simulator.commands.AutoCommandsSim;
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
import java.util.Locale;

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
@Autonomous(name = "Low Hive Start", group = "Auto")
public class LowHiveStart extends CommandOpMode {

    private MecanumDriveSubsystem drive;
    private PollenShooterSubsystem pss;
    private NectarShooterSubsystem nss;
    private IntakeSubsystem intake;

    private VisionSubsysytem vss;
    private Follower follower;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private Pose startPose, controlPointPose, flowerApproachPose, flowerPickupPose, secondShootPose, parkControlPointPose, parkPose;
    private Path approachFlower, pickupFlower, moveForSecondShoot, parkAfterSecondShoot, park;
    private AutoCommandsSim acSim;

    private AutoCommands ac;

    private GetRPMFromDistance grpmpollenfd;


    private ElapsedTime autoTimer;

    private int seqStep;
    private double distancetoTarget;
    private double targetRPMFromDistance;

    static String f3(double v) {
        return String.format(Locale.US, "%.3f", v);
    }

    static String f1(double v) {
        return String.format(Locale.US, "%.1f", v);
    }

    static String f0(double v) {
        return String.format(Locale.US, "%.0f", v);
    }

    static String f3M2I(double v) {
        return String.format(Locale.US, "%.3f", v * 39.37);
    }


    @Override
    public void initialize() {
        super.reset();

        super.reset();


        drive = new MecanumDriveSubsystem(this.hardwareMap);


        vss = new VisionSubsysytem(this);

        pss = new PollenShooterSubsystem(this.hardwareMap);

        nss = new NectarShooterSubsystem(this.hardwareMap);

        ac = new AutoCommands(drive, pss, nss);


        intake = new IntakeSubsystem(this.hardwareMap);


        GlobalData.autoHasRun = true;

        autoTimer = new ElapsedTime();

        startPose = forAlliance(FieldConstants.redScoringLoHiveStartPose, GlobalData.isBlueAlliance());
        flowerApproachPose = forAlliance(FieldConstants.scoringWallFlowerApproachPose, GlobalData.isBlueAlliance());
        flowerPickupPose = forAlliance(FieldConstants.scoringWallFlowerPickupPose, GlobalData.isBlueAlliance());
        secondShootPose = forAlliance(FieldConstants.scoringWallFlowerApproachPose, GlobalData.isBlueAlliance());
        parkPose = forAlliance(FieldConstants.redScoringParkPose, GlobalData.isBlueAlliance());


        controlPointPose = GlobalData.isBlueAlliance()
                ? FieldConstants.toBlueXY(FieldConstants.scoringWallFlowerApproachControlPointPose)
                : FieldConstants.scoringWallFlowerApproachControlPointPose;

        parkControlPointPose = GlobalData.isBlueAlliance()
                ? FieldConstants.toBlueXY(FieldConstants.lowHiveStartControlPointPose)
                : FieldConstants.lowHiveStartControlPointPose;


        follower = Constants.createFollower(this.hardwareMap);


        follower.setPose(startPose);

        buildPaths();

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
        return

                Commands.sequence(

                        new CheckIfOdometryZoneTagsSeen(vss, true).withTimeout(20000),

                        checkSequenceStepCommand(),

                        Commands.defer(() -> new ConditionalCommand(
                                        Commands.sequence(
                                                pss.setTargetRPMCommand(PollenShooterSubsystem.autoShootRPM),
                                                shoot(),
                                                pickup(),
                                                moveForSecondShoot(),
                                                pss.setTargetRPMCommand(PollenShooterSubsystem.autoSecondShootRPM),
                                                shoot(),
                                                parkAfterSecondShoot()),
                                        park(),
                                        () -> vss.getZoneTagsSeen()),
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
                checkSequenceStepCommand(),
                new FollowPathCommand(follower, approachFlower),
                new WaitCommand(250),
                checkSequenceStepCommand());
    }

    public Command park() {
        return new FollowPathCommand(follower, park, false);
    }

    public Command parkAfterSecondShoot() {
        return new FollowPathCommand(follower, parkAfterSecondShoot, false);
    }

    public Command moveForSecondShoot() {

        return Commands.sequence(
                new FollowPathCommand(follower, moveForSecondShoot, false),
                checkSequenceStepCommand(),
                Commands.parallel(
                        intake.stopIntakeCommand(),
                        intake.armUpCommand()),
                new AlignToPoseCommand(drive));


    }


    private Command shoot() {
        return Commands.sequence(
                checkSequenceStepCommand(),
                acSim.autoShootPollenCommand());
    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();
        autoTimer.reset();
        while (!isStopRequested() && opModeIsActive() && GlobalData.allianceIsConfirmed && autoTimer.seconds() <= 30) {

            if (GlobalData.isRedAlliance() || GlobalData.isBlueAlliance()) {
                run();

                follower.update();


                telemetryM.addData("Auto Timer", autoTimer.seconds());
                telemetryM.addData("Follower Is Busy", follower.isBusy());
                telemetryM.addData("Shooting", pss.shooterServoTargetPosition == PollenShooterSubsystem.shootPosition);
                telemetryM.addData("Intake Running", intake.intakeRunning);
                telemetryM.addData("Intake Arm Down", intake.getArmPosition() == IntakeSubsystem.downPosition);

                telemetryM.addData("SeqStep", seqStep);

                telemetryM.update(telemetry);
            }
        }


        reset();
    }

    public void buildPaths() {
        park = buildBezierCurve(startPose, parkControlPointPose, parkPose);
        approachFlower = buildBezierCurve(startPose, controlPointPose, flowerApproachPose);
        pickupFlower = buildBezierLine(flowerApproachPose, flowerPickupPose);
        moveForSecondShoot = buildBezierLine(flowerPickupPose, secondShootPose);
        parkAfterSecondShoot = buildBezierLine(secondShootPose, parkPose);

    }

    Path buildBezierLine(Pose start, Pose end) {
        return PedroPaths.line(start, end);
    }

    Path buildBezierCurve(Pose start, Pose control, Pose end) {
        return PedroPaths.curve(start, control, end);
    }


    Path buildBezierCurve(Pose start, Pose control, Pose control1, Pose end) {
        return PedroPaths.curve(start, control, control1, end);
    }

    Path buildBezierCurve(Pose start, Pose control, Pose control1, Pose control2, Pose end) {
        return PedroPaths.curve(start, control, control1, control2, end);
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
