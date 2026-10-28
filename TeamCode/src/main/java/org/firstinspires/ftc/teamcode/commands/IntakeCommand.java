package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.CV.BiobuzzBallPipeline_1;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;
import org.firstinspires.ftc.teamcode.utils.GlobalData;

import java.util.function.DoubleSupplier;

public class IntakeCommand extends CommandBase {

    private MecanumDriveSubsystem drive;
    private TelemetryManager telemetryM;
    private IntakeSubsystem intake;
    private WebcamSubsystem wcs;
    private PanelsCameraStream pcs;

    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;

    public static double kP = .5;
    public static double kI = 0.0;
    public static double kD = 0.0;
    private final PIDController headingController = new PIDController(kP, kI, kD);
    public static double deadbandRad = Math.toRadians(1.0);
    /**
     * Limit on turn power so aiming never saturates the motors.
     */
    public static double maxTurn = .5;
    /**
     * Added to the computed angle, e.g. Math.PI if your shooter faces the back of the robot.
     */
    public static double headingOffsetRad = 0.0;
    private GamepadEx driverGamepad;

    public IntakeCommand(MecanumDriveSubsystem drive, IntakeSubsystem intake, WebcamSubsystem wcs, DoubleSupplier forward, DoubleSupplier strafe) {
        this.drive = drive;
        this.intake = intake;
        this.wcs = wcs;
        this.forward = forward;
        this.strafe = strafe;
    }

    @Override
    public void initialize() {

        pcs = PanelsCameraStream.INSTANCE;
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pcs.startStream(wcs.webcam, 20);
        headingController.reset();
        headingController.setPID(kP, kI, kD);
        drive.startTeleopDrive();
    }


    @Override
    public void execute() {
        intake.pollenCount = wcs.getCount(BiobuzzBallPipeline_1.BallColor.YELLOW);
        intake.redNectarCount = wcs.getCount(BiobuzzBallPipeline_1.BallColor.RED);
        intake.blueNectarCount = wcs.getCount(BiobuzzBallPipeline_1.BallColor.BLUE);

        if (GlobalData.isRedAlliance()) {
            intake.allianceNectarCount = intake.redNectarCount;
            intake.opponemtNectarCount = intake.blueNectarCount;
        } else {
            intake.allianceNectarCount = intake.blueNectarCount;
            intake.opponemtNectarCount = intake.redNectarCount;
        }

        BiobuzzBallPipeline_1.Ball nearest = wcs.getNearest();
    }


    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
