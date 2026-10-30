package org.firstinspires.ftc.teamcode.simulator.opmodes.auto;

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
import org.firstinspires.ftc.teamcode.simulator.SimulatorConstants;
import org.firstinspires.ftc.teamcode.simulator.commands.AlignToPoseCommandSim;
import org.firstinspires.ftc.teamcode.simulator.commands.AutoCommandsSim;
import org.firstinspires.ftc.teamcode.simulator.drivetrains.MecanumDriveSubsystemSimulation;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NectarShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PollenShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsysytem;
import org.firstinspires.ftc.teamcode.utils.Configurables;
import org.firstinspires.ftc.teamcode.utils.FieldConstants;
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
@Autonomous(name = "Sim Low Hive Start", group = "Sim")
public class LowHiveStartSim extends CommandOpMode {

    private MecanumDriveSubsystemSimulation driveSim;
    private PollenShooterSubsystem pss;
    private NectarShooterSubsystem nss;
    private IntakeSubsystem intake;

    private VisionSubsysytem vss;
    private Follower follower;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private Pose startPose, controlPointPose, flowerApproachPose, flowerPickupPose, secondShootPose, parkControlPointPose, parkPose;
    private Path parkPath, approachFlowerPath, pickupFlowerPath, moveForSecondShootPath, parkAfterSecondShootPath;
    private AutoCommandsSim acSim;

    private AutoCommandsSim ac;



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


        driveSim = new MecanumDriveSubsystemSimulation(this);


        vss = new VisionSubsysytem(this);

        pss = new PollenShooterSubsystem(this.hardwareMap);

        nss = new NectarShooterSubsystem(this.hardwareMap);

        acSim = new AutoCommandsSim(driveSim, pss, nss);

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


        follower = SimulatorConstants.createSimulatedFollower(driveSim);

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

                        Commands.waitMillis(1000),
                       // Commands.runOnce(()->Configurables.hiveRaisedSim=true),

                        Commands.defer(() -> new ConditionalCommand(
                                        Commands.sequence(
                                                pss.setTargetRPMCommand(PollenShooterSubsystem.autoShootRPM),
                                                shoot(),
                                                checkSequenceStepCommand(),
                                                pickup(),
                                                checkSequenceStepCommand(),
                                                moveForSecondShoot(),
                                                checkSequenceStepCommand(),
                                                pss.setTargetRPMCommand(PollenShooterSubsystem.autoSecondShootRPM),
                                                shoot(),
                                                checkSequenceStepCommand(),
                                                parkAfterSecondShoot(),
                                                checkSequenceStepCommand()),
                                        park(),
                                        () -> Configurables.hiveRaisedSim),
                                Collections.emptyList()));
    }


    public Command pickup() {
        return Commands.sequence(
                Commands.parallel(
                        new FollowPathCommand(follower, approachFlowerPath),
                        intake.runIntakeCommand(),
                        intake.armDownCommand()),
                // checkSequenceStepCommand(),
                new WaitCommand(1000),
                new FollowPathCommand(follower, pickupFlowerPath).setGlobalMaxPower(0.25),
                new WaitCommand(1500),
                // checkSequenceStepCommand(),
                new FollowPathCommand(follower, approachFlowerPath),
                new WaitCommand(250));

    }

    public Command park() {
        return new FollowPathCommand(follower, parkPath, false);
    }

    public Command parkAfterSecondShoot() {
        return new FollowPathCommand(follower, parkAfterSecondShootPath, false);
    }

    public Command moveForSecondShoot() {
        return Commands.sequence(
                new FollowPathCommand(follower, moveForSecondShootPath,false),
                  checkSequenceStepCommand(),
                Commands.parallel(
                        intake.stopIntakeCommand(),
                        intake.armUpCommand()),
                checkSequenceStepCommand(),
                // Pedro 3 holds the end pose and re-commands the drivetrain every update, which
                // would override the align command's turn. Release the hold first.
                Commands.waitUntil(()->!follower.isBusy()),
                Commands.runOnce(() -> follower.stop()),
                Commands.defer(() -> new AlignToPoseCommandSim(driveSim).asProxy(),
                        Collections.emptyList()));

    }


    private Command shoot() {
        return Commands.sequence(

                pss.setTargetRPMCommand(PollenShooterSubsystem.autoShootRPM),
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

                if (!follower.idle()) follower.update();

                distancetoTarget = getDistanceToTarget(GlobalData.getActiveHiveTargetPose(), driveSim.getOdometry().getRobotPose());


                //   pss.setTargetRPM(targetRPMFromDistance);

                distancetoTarget = getDistanceToTarget(GlobalData.getActiveHiveTargetPose(), driveSim.getOdometry().getRobotPose());



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
        parkPath = PedroPaths.curve(startPose, parkControlPointPose, parkPose);
        approachFlowerPath = PedroPaths.curve(startPose, controlPointPose, flowerApproachPose);
        pickupFlowerPath = PedroPaths.line(flowerApproachPose, flowerPickupPose);
        moveForSecondShootPath = PedroPaths.curve(flowerPickupPose, secondShootPose);
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
