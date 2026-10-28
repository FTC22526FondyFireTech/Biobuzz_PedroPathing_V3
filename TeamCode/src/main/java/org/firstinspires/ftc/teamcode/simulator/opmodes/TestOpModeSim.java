package org.firstinspires.ftc.teamcode.simulator.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.utils.Configurables;
import org.firstinspires.ftc.teamcode.utils.PedroPaths;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;

import org.firstinspires.ftc.teamcode.simulator.SimulatorConstants;
import org.firstinspires.ftc.teamcode.simulator.commands.AlignToPoseCommandSim;
import org.firstinspires.ftc.teamcode.simulator.commands.DriveSimCommand;
import org.firstinspires.ftc.teamcode.simulator.commands.DriveToPoseCommandSim;
import org.firstinspires.ftc.teamcode.simulator.drivetrains.MecanumDriveSubsystemSimulation;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NectarShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PollenShooterSubsystem;
import org.firstinspires.ftc.teamcode.utils.FieldConstants;
import org.firstinspires.ftc.teamcode.utils.GlobalData;

import java.util.Arrays;
import java.util.Locale;

//@Autonomous(name = "Blank")

@TeleOp(name = "TestOpMode", group = "Sim")

public class TestOpModeSim extends CommandOpMode {

    TelemetryManager telemetryM;
    GamepadEx driverGamepad;

    private Follower follower;

    MecanumDriveSubsystemSimulation driveSim;
    IntakeSubsystem intake;
    PollenShooterSubsystem pollenShooter;

    NectarShooterSubsystem nectarShooter;

    Path myPath;

    Pose startPose;
    Pose controlPointPose;

    Pose parkPoseFromStart;

    Path parkFromStart;


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


        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


        driverGamepad = new GamepadEx(gamepad1);

        driveSim = new MecanumDriveSubsystemSimulation(this);
        telemetryM.addData("RC", driveSim.isRobotCentric());
        telemetryM.update(telemetry);
        follower = SimulatorConstants.createSimulatedFollower(driveSim);
        driveSim.setDefaultCommand(new DriveSimCommand(
                driveSim,
                () -> driverGamepad.getLeftY(),
                () -> driverGamepad.getLeftX(),
                () -> driverGamepad.getRightX(), () -> true));

        intake = new IntakeSubsystem(this.hardwareMap);
        pollenShooter = new PollenShooterSubsystem(this.hardwareMap);
        nectarShooter = new NectarShooterSubsystem(this.hardwareMap);

        GlobalData.setRedAlliance();
        startPose = FieldConstants.redAudienceHiHiveStartPose;
        controlPointPose = FieldConstants.redAllianceParkfromStartControlPointPose;

        parkPoseFromStart = FieldConstants.redAudienceParkPoseFromStart;

        myPath = buildBezierLine(new Pose(0, 0, 0), new Pose(10, 10, 0));
        parkFromStart = buildBezierCurve(startPose, controlPointPose, parkPoseFromStart);

        //  parkFromStart.setDecelerationType(Path.DecelerationType.GLOBAL);

        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new AlignToPoseCommandSim(driveSim, () -> driverGamepad.getLeftY(), () -> driverGamepad.getLeftX()));


        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        Commands.defer(() ->
                                new DriveToPoseCommandSim(driveSim, GlobalData.getActiveStartPose()), Arrays.asList()));

        driverGamepad.getGamepadButton(GamepadKeys.Button.BACK)
                .whileActiveOnce(GlobalData.toggleAllianceCommand());


//        driverGamepad.getGamepadButton(GamepadKeys.Button.A)
//                .whenPressed(intake.armUpCommand());
//        driverGamepad.getGamepadButton(GamepadKeys.Button.B)
//                .whenPressed(intake.armDownCommand());


//                //.whenPressed(shooter2.setMotorVelocityModeCommand())
//                .whenHeld(pollenShooter.jogPollenShooterCommand(0.5))
//                .whenReleased(pollenShooter.stopPollenShooterCommand());
//
        driverGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(driveSim.getOdometry().setPoseCommand(startPose));


        driverGamepad.getGamepadButton(GamepadKeys.Button.Y).whileActiveOnce(
                //Commands.sequence(

                new FollowPathCommand(follower, parkFromStart, true));

        driverGamepad.getGamepadButton(GamepadKeys.Button.A).whileActiveOnce(
                Commands.runOnce(() -> follower.setPose(startPose)));

    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            run();
            // A held end pose would otherwise override the sticks; let the driver take back control.
            if (follower.holding()
                    && (Math.abs(driverGamepad.getLeftY()) > 0.1
                    || Math.abs(driverGamepad.getLeftX()) > 0.1
                    || Math.abs(driverGamepad.getRightX()) > 0.1)) {
                follower.stop();
            }
            // Pedro 3: an idle follower calls drivetrain.stop() on every update, which would zero the
            // motors the gamepad drive command just set. Only update while following/holding.
            if (!follower.idle()) follower.update();

            driveSim.showTelemetry(telemetryM);

        double distToTarget = getDistanceToTarget(GlobalData.getActiveHiveTargetPose(), driveSim.getOdometry().getRobotPose());
        //  pollenShooter.setTargetRPMFromDistance(distToTarget);
        double pollenRPM = pollenShooter.getTargetRPM();
        double nectarRPM = nectarShooter.getTargetRPMFromDistance(distToTarget);
          telemetryM.addData("ENDPOSX", parkFromStart.endPose().x());
//            telemetryM.addData("ENDPOSX=Y", parkFromStart.endPose().y());
//            telemetryM.addData("ENDPOSH", parkFromStart.endPose().heading());
//            telemetryM.addData("ROBPOSX", driveSim.getOdometry().getX());
//            telemetryM.addData("ROBPOSY", driveSim.getOdometry().getY());
     //       telemetryM.addData("ROBPOSH", driveSim.getOdometry().getHeading());
//
//            telemetryM.addData("DistRem", follower.remainingDistance());
//            telemetryM.addData("PathCompl", follower.completion());


//            telemetryM.addData("InakeArmPosition", intake.getArmPosition());
//            telemetryM.addData("Dist2Tgt,in", f1(distToTarget));
//            telemetryM.addData("Tgt Pollen RPM", f0(pollenRPM));
//            telemetryM.addData("Tgt Nectar RPM", f0(nectarRPM));
        telemetryM.addData("Follower Is Busy", follower.isBusy());
        telemetryM.update(telemetry);

    }

    reset();
}

        Path buildBezierLine(Pose start, Pose end) {
            return PedroPaths.line(start, end);
        }

        Path buildBezierCurve(Pose start, Pose control, Pose end) {
            return PedroPaths.curve(start, control, end);
        }

        public double getDistanceToTarget(Pose targetPose, Pose robotPose) {
            double XDiff = targetPose.x() - robotPose.x();
            double YDiff = targetPose.y() - robotPose.y();
            return Math.hypot(XDiff, YDiff);
        }

}