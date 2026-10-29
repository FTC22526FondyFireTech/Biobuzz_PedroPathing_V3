package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.commands.AlignToPoseCommand;
import org.firstinspires.ftc.teamcode.commands.CheckIfOdometryZoneTagsSeen;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPoseCommand;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NectarShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PollenShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsysytem;
import org.firstinspires.ftc.teamcode.utils.Configurables;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.FieldConstants;
import org.firstinspires.ftc.teamcode.utils.GPTrig;
import org.firstinspires.ftc.teamcode.utils.GlobalData;
import org.firstinspires.ftc.teamcode.utils.PedroPaths;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Teleop", group = "Teleop")
//@Disabled

public class TeleopOpmode extends CommandOpMode {

    TelemetryManager telemetryM;
    GamepadEx driverGamepad;

    GPTrig leftTrigger;
    GPTrig rightTrigger;

    GamepadEx codriverGamepad;
    MecanumDriveSubsystem drive;
    VisionSubsysytem vss;
    IntakeSubsystem intake;

    PollenShooterSubsystem pss;
    NectarShooterSubsystem nss;

    private Pose redResetOdometry =
            new Pose(FieldConstants.robotWidth / 2,
                    FieldConstants.fieldLength - FieldConstants.robotLength / 2,
                    Math.PI / 2);
    private Pose blueResetOdometry = new Pose(
            FieldConstants.fieldWidth - FieldConstants.robotWidth / 2,
            FieldConstants.robotLength / 2, -Math.PI / 2);

    private Pose resetOdometry = Pose.zero();

    private Follower follower;

    static String f3(double v) {
        return String.format(Locale.US, "%.3f", v);
    }

    static String f3M2I(double v) {
        return String.format(Locale.US, "%.3f", v * 39.37);
    }

    Timing.Timer readCamera;
    long readMilliSecs = 250;

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryM.update(telemetry);

        readCamera = new Timing.Timer(readMilliSecs, TimeUnit.MILLISECONDS);
        readCamera.start();


        if (!GlobalData.isBlueAlliance() && !GlobalData.isRedAlliance())
            GlobalData.setRedAlliance();

        if (GlobalData.isRedAlliance()) {
            resetOdometry = redResetOdometry;
        } else {
            resetOdometry = blueResetOdometry;
        }

        driverGamepad = new GamepadEx(gamepad1);
        codriverGamepad = new GamepadEx(gamepad2);

        drive = new MecanumDriveSubsystem(this.hardwareMap, Pose.zero());

        drive.setDefaultCommand(new DriveCommand(drive,
                () -> driverGamepad.getLeftY(),
                () -> -driverGamepad.getLeftX(),
                () -> driverGamepad.getRightX()));

        if (GlobalData.autoHasRun) {
            drive.setPose(GlobalData.robotPose);
            GlobalData.autoHasRun = false;
        }
        vss = new VisionSubsysytem(this);

        pss = new PollenShooterSubsystem(this.hardwareMap);

        nss = new NectarShooterSubsystem(this.hardwareMap);

        intake = new IntakeSubsystem(this.hardwareMap);

        follower = Constants.createFollower(this.hardwareMap);

        driverGamepad.getGamepadButton(GamepadKeys.Button.BACK)
                .whileActiveOnce(GlobalData.toggleAllianceCommand());

        driverGamepad.getGamepadButton(GamepadKeys.Button.START)
                .whileActiveOnce(drive.setPoseCommand(resetOdometry));


        //Triggers

        leftTrigger = new GPTrig(driverGamepad, GPTrig.Side.LEFT);
        rightTrigger = new GPTrig(driverGamepad, GPTrig.Side.RIGHT);

        leftTrigger.whenActive(intake.runIntakeCommand())
                .whenInactive(intake.stopIntakeCommand())
                .whenInactive(intake.armUpCommand());

        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(intake.toggleArmPositionCommand());

        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        Commands.parallel(
                                pss.runPollenShooterAtVelocityCommand(),
                                nss.runNectarShooterAtVelocityCommand()));

        rightTrigger.whenActive(
                        Commands.parallel(
                                pss.shootPollenCommand(),
                                nss.shootNectarCommand()))
                .whenInactive(
                        Commands.parallel(
                                pss.holdPollenCommand(),
                                nss.holdNectarCommand()));

        driverGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(Commands.parallel(
                        pss.stopPollenShooterCommand(),
                        nss.stopNectarShooterCommand()));

        driverGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(Commands.defer(() ->
                                new AlignToPoseCommand(drive),
                        Collections.emptyList()));


        driverGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(
                        Commands.defer
                                (() -> Commands.sequence(
                                                Commands.runOnce(() -> follower.setPose(drive.getPose())),
                                                new FollowPathCommand(follower,
                                                        PedroPaths.line(drive.getPose(), GlobalData.getActiveStartPose()),
                                                        false),
                                                new AlignToPoseCommand(drive)),
                                        Collections.emptyList()));

        //  driverGamepad.getGamepadButton(GamepadKeys.Button.a)

        double rpmChange = 100;
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(pss.changePollenTargetRPMCommand(rpmChange));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(pss.changePollenTargetRPMCommand(-rpmChange));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(nss.changeNectarTargetRPMCommand(rpmChange));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(nss.changeNectarTargetRPMCommand(-rpmChange));
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            run();

            telemetryM.addData("IsBusy", follower.isBusy());

            double dist = 0;
            double angle = 0;
            dist = getDistanceToTarget(GlobalData.getActiveHiveTargetPose(), drive.getPose());
            angle = getAngleToTarget(GlobalData.getActiveHiveTargetPose(), drive.getPose());

            telemetryM.addData("Target Distance Inches", dist);
            telemetryM.addData("Target Angle Degrees", angle);
            telemetryM.addData("Odometry Zone", GlobalData.getCurrentOdometryZone());
            telemetryM.addData("Vision Zone", vss.isInCurrentZone(vss.getFiducialResults()));

            if (Configurables.showDriveTelemetry)
                drive.showTelemetry(telemetryM);
            if (Configurables.showPollenTelemetry)
                pss.showTelemetry(telemetryM);
            if (Configurables.showNectarTelemetry)
                nss.showTelemetry(telemetryM);
            if (Configurables.showIntakeTelemetry)
                intake.showTelemetry(telemetryM);
            if (Configurables.showVisionTelemetry)
                vss.showTelemetry(telemetryM);

            telemetryM.update(telemetry);

            if (readCamera.done()) {
                boolean zoneCheck = checkOdometryZoneSameAsVisionZone(vss);
                readCamera.start();
            }
        }
        reset();
    }


    public double getAngleToTarget(Pose targetPose, Pose robotPose) {
        double XDiff = targetPose.x() - robotPose.x();
        double YDiff = targetPose.y() - robotPose.y();
        return Math.toDegrees(Math.atan2(YDiff, XDiff));
    }

    public double getDistanceToTarget(Pose targetPose, Pose robotPose) {
        double XDiff = targetPose.x() - robotPose.x();
        double YDiff = targetPose.y() - robotPose.y();
        return Math.hypot(XDiff, YDiff);
    }

    public boolean checkOdometryZoneSameAsVisionZone(VisionSubsysytem vss) {
        List<LLResultTypes.FiducialResult> llResults = vss.getFiducialResults();
        vss.updateCurrentTagZone(llResults);
        return GlobalData.getCurrentOdometryZone() == vss.getCurrentTagZone();
    }


}