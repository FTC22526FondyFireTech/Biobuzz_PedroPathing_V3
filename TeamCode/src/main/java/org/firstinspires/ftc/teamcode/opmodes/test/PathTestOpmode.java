package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.utils.PedroPaths;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.simulator.SimulatorConstants;
import org.firstinspires.ftc.teamcode.simulator.commands.DriveSimCommand;
import org.firstinspires.ftc.teamcode.simulator.drivetrains.MecanumDriveSubsystemSimulation;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.Configurables;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.GlobalData;

import java.util.Arrays;
import java.util.Collections;

//@Autonomous(name = "Blank")
@TeleOp(name = "Path Test")
@Disabled

public class PathTestOpmode extends CommandOpMode {
    TelemetryManager telemetryM;
    GamepadEx driverGamepad;
    MecanumDriveSubsystem drive;
    private MecanumDriveSubsystemSimulation driveSim;

    // Pose Factory
    private final Pose blueStartPose1 = new Pose(12, 12, Math.PI / 2);
    private final Pose blueStartPose2 = new Pose(59, 12, Math.PI / 2);
    private final Pose blueTestPose1 = new Pose(12, 36, Math.PI / 2);
    private final Pose blueTestPose2 = new Pose(59,82, Math.PI);
    private final Pose blueIntakePose = new Pose(20, 82, Math.PI);
    private final Pose blueShootPose = new Pose(35, 106, 3 * Math.PI / 4);

    // POSES
    private Pose startPose1 = Pose.zero();
    private Pose startPose2 = new Pose(24,48);
    private Pose testPose1 = new Pose(36,80);
    private Pose testPose2 = Pose.zero();
    private Pose intakePose = Pose.zero();
    private Pose shootPose = Pose.zero();

    // AUTOS
    private Path forwardOneTile;
    private Path intakePath;
    private Path shootPath;

    // OTHER
    private Follower follower;

    @Override
    public void initialize() {
        GlobalData.setBlueAlliance();
        driverGamepad = new GamepadEx(gamepad1);

        if (!Configurables.doSimulation) {
            drive = new MecanumDriveSubsystem(this.hardwareMap, Pose.zero());
            drive.setDefaultCommand(new DriveCommand(drive,
                    () -> driverGamepad.getLeftY(),
                    () -> -driverGamepad.getLeftX(),
                    () -> driverGamepad.getRightX()));
           // intake = new IntakeSubsystem(this.hardwareMap);
        } else {
            driveSim = new MecanumDriveSubsystemSimulation(this);
            driveSim.setDefaultCommand(new DriveSimCommand(
                    driveSim,
                    () -> driverGamepad.getLeftY(),
                    () -> driverGamepad.getLeftX(),
                    () -> driverGamepad.getRightX(), () -> true));
           // intake = null; // doesn't run intake in simulation
        }

        if (!Configurables.doSimulation)
            follower = Constants.createFollower(this.hardwareMap);
        else
            follower = SimulatorConstants.createSimulatedFollower(driveSim);

        setAlliancePaths();

        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileActiveOnce(
                Commands.sequence(
                        GlobalData.toggleAllianceCommand(),
                        Commands.runOnce(this::setAlliancePaths)));

        if (!Configurables.doSimulation) {
            driverGamepad.getGamepadButton(GamepadKeys.Button.Y)
                    .whileActiveOnce(drive.resetPoseCommand());
        } else {
            driverGamepad.getGamepadButton(GamepadKeys.Button.Y)
                    .whileActiveOnce(driveSim.getOdometry().resetPoseCommand());

        }

        if (!Configurables.doSimulation) {
            driverGamepad.getGamepadButton(GamepadKeys.Button.A)
                    .whileActiveOnce(
                            Commands.defer(() ->
                                    drive.setPoseCommand(startPose2), Arrays.asList(drive)));
        } else {
            driverGamepad.getGamepadButton(GamepadKeys.Button.A)
                    .whileActiveOnce(
                            Commands.defer(() ->
                                    driveSim.getOdometry().setPoseCommand(startPose2), Arrays.asList(driveSim)));

        }

        // forwardOneTile Path binding
        driverGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whileActiveOnce(
                        Commands.defer
                                (() -> Commands.sequence(
                                        Commands.runOnce(() -> follower.setPose(startPose1)),
                                        new FollowPathCommand(follower, forwardOneTile, false)), Collections.emptyList()));

        // intake path binding
        driverGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whileActiveOnce(
                        Commands.defer
                                (() -> Commands.sequence( // sequential
                                        Commands.runOnce(() -> follower.setPose(startPose1)),
                                        new FollowPathCommand(follower,
                                                intakePath,
                                                false)),
                                                Arrays.asList()));

        // shoot path binding
        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileActiveOnce(
                        Commands.defer
                                (() -> Commands.sequence( // sequential
                                                Commands.runOnce(() -> follower.setPose(startPose2)),
                                                new FollowPathCommand(follower,
                                                        shootPath,
                                                        false)),
                                        Arrays.asList()));


        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryM.update(telemetry);


    }

    private void setAlliancePaths() {
        if (GlobalData.isRedAlliance()) {
            startPose1 = flipBlueToRedPose(blueStartPose1);
            testPose1 = flipBlueToRedPose(blueTestPose1);
            testPose2 = flipBlueToRedPose(blueTestPose2);
            startPose2 = flipBlueToRedPose(blueStartPose2);
            intakePose = flipBlueToRedPose(blueIntakePose);
            shootPose = flipBlueToRedPose(blueShootPose);


        } else {
            startPose1 = blueStartPose1;
            testPose1 = blueTestPose1;
            testPose2 = blueTestPose2;
            startPose2 = blueStartPose2;
            intakePose = blueIntakePose;
            shootPose = blueShootPose;
        }


        buildPaths();
    }


    public Pose flipBlueToRedPose(Pose blue) {
        double x = blue.x();
        double y = blue.y();
        x = SimulatorConstants.width - x;
        double h = blue.heading();
        return new Pose(x, y, Math.PI - h);
    }

    private void buildPaths() {

        // auto that moves the robot forward 1 tile
        forwardOneTile = PedroPaths.line(startPose1, testPose1);

        if(true) {
            // auto that tests hardware with paths
            intakePath = PedroPaths.curve(startPose1, new Pose(11.226, 64.416), testPose2);

            // auto that tests strafing, control points, curves, and hardware
            shootPath = com.pedropathing.api.Paths.path(
                    // (3,1) to (3,4)
                    com.pedropathing.api.Paths.line(startPose2, testPose2)
                            .linear(startPose1.heading(), testPose2.heading()),
                    // (3,4) to (1.35,4)
                    PedroPaths.line(testPose2, intakePose),
                    // (1.35,4) to (2,5), control point in the middle
                    PedroPaths.curve(intakePose, new Pose(36.478, 89.162), shootPose));
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            run();

            if (!Configurables.doSimulation) {
                drive.setRobotCentric(driverGamepad.getGamepadButton(
                        GamepadKeys.Button.RIGHT_BUMPER).get());
                drive.showTelemetry(telemetryM);
            } else {
                driveSim.setRobotCentric(driverGamepad.getGamepadButton(
                        GamepadKeys.Button.RIGHT_BUMPER).get());
                driveSim.showTelemetry(telemetryM);
            }

            //if (Configurables.doSimulation)
            // Pedro 3: an idle follower calls drivetrain.stop() on every update, which would zero the
            // motors the gamepad drive command just set. Only update while following/holding.
            if (!follower.idle()) follower.update();



            telemetryM.update(telemetry);
        }
        reset();
    }

}