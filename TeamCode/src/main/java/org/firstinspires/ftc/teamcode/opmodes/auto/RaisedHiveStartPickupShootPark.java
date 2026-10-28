package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.utils.PedroPaths;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.simulator.SimulatorConstants;
import org.firstinspires.ftc.teamcode.simulator.drivetrains.MecanumDriveSubsystemSimulation;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsysytem;
import org.firstinspires.ftc.teamcode.utils.Configurables;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.FieldConstants;
import org.firstinspires.ftc.teamcode.utils.GlobalData;

/**
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
@Autonomous(name = "Raised Hive Start Pickup, Shoot, Park", group = "Auto")
public class RaisedHiveStartPickupShootPark extends CommandOpMode {

    private MecanumDriveSubsystem drive;
    private MecanumDriveSubsystemSimulation driveSim;

    private VisionSubsysytem vss;
    private Follower follower;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private Pose startPose, flowerApproachPose, flowerPickupPose,
            secondShootPose, secondFlowerApproachPose, secondFlowerPickupPose, thirdShootPose, parkPose;
    private Path approachFlower, pickupFlower, secondShoot, secondApproach, secondPickup, thirdShoot, park;

    private boolean isBlueAlliance=false;

    boolean allianceSelected;
    boolean choicesComplete;
    private boolean allianceIsSelected;


    @Override
    public void initialize() {
        super.reset();

        isBlueAlliance = GlobalData.isBlueAlliance();

        if (!Configurables.doSimulation)
            drive = new MecanumDriveSubsystem(this.hardwareMap);
        else
            driveSim = new MecanumDriveSubsystemSimulation(this);

        vss = new VisionSubsysytem(this);

        //  GlobalData.selectAlliance(this);
        GlobalData.autoHasRun = true;

        startPose = forAlliance(FieldConstants.redAudienceHiHiveStartPose, isBlueAlliance);
        flowerApproachPose = forAlliance(FieldConstants.redAllianceWallFlowerApproachPose, isBlueAlliance);
        flowerPickupPose = forAlliance(FieldConstants.redAllianceWallFlowerPickupPose, isBlueAlliance);
        secondShootPose = forAlliance(FieldConstants.redScoringSecondShootPose, isBlueAlliance);
        secondFlowerApproachPose = forAlliance(FieldConstants.scoringWallFlowerApproachPose, isBlueAlliance);
        secondFlowerPickupPose = forAlliance(FieldConstants.scoringWallFlowerPickupPose, isBlueAlliance);


        parkPose = forAlliance(FieldConstants.redAudienceParkPose, isBlueAlliance);


        // Only this line differs from PedroAutoSample.initialize() - everything below is
        // identical Follower/Path/FollowPathCommand usage.

        if (!Configurables.doSimulation)
            follower = Constants.createFollower(this.hardwareMap);
        else
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

                        new ConditionalCommand(
                                Commands.sequence(
                                        new WaitCommand(500),
                                        new FollowPathCommand(follower, approachFlower),
                                        new WaitCommand(500),
                                        new FollowPathCommand(follower, pickupFlower).setGlobalMaxPower(0.5),
                                        new WaitCommand(500),
                                        new FollowPathCommand(follower, approachFlower),
                                        new WaitCommand(500),
                                        new FollowPathCommand(follower, park, false)),

                                new FollowPathCommand(follower, park, false),

                                () -> vss.getZoneTagsSeen() || Configurables.doSimulation));


    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        while (!isStopRequested() && opModeIsActive() && GlobalData.allianceIsConfirmed) {
            run();

            follower.update();

            telemetryM.addData("X", follower.pose().x());
            telemetryM.addData("Y", follower.pose().y());
            telemetryM.addData("Heading", Math.toDegrees(follower.pose().heading()));
            telemetryM.addData("Busy", follower.isBusy());

            telemetryM.update(telemetry);
        }
        reset();
    }

    public void buildPaths() {

        approachFlower = buildBezierCurve(startPose, FieldConstants.scoringWallFlowerApproachControlPointPose, flowerApproachPose);
        pickupFlower = buildBezierLine(flowerApproachPose, flowerPickupPose);
        park = buildBezierLine(flowerPickupPose, parkPose);

    }

    Path buildBezierLine(Pose start, Pose end) {
        return PedroPaths.line(start, end);
    }

    Path buildBezierCurve(Pose start, Pose control, Pose end) {
        return PedroPaths.curve(start, control, end);
    }


}
