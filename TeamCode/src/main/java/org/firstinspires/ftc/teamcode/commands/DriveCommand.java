package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.GlobalData;

import java.util.function.DoubleSupplier;

/**
 * Default teleop drive command. Continuously feeds gamepad stick input to the
 * {@link MecanumDriveSubsystem} until interrupted (e.g. by an auto-align command
 * taking over the subsystem). Wire this up as the drive subsystem's default command:
 *
 * <pre>{@code
 * drive.setDefaultCommand(new DriveCommand(
 *     drive,
 *     () -> -gamepad1.left_stick_y,
 *     () -> -gamepad1.left_stick_x,
 *     () -> -gamepad1.right_stick_x,
 *     () -> !gamepad1.left_bumper // hold left bumper for robot-centric
 * ));
 * }</pre>
 */
public class DriveCommand extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier turn;

    public DriveCommand(MecanumDriveSubsystem drive,
                        DoubleSupplier forward,
                        DoubleSupplier strafe,
                        DoubleSupplier turn
    ) {
        this.drive = drive;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.startTeleopDrive();
    }

    @Override
    public void execute() {
        if (GlobalData.isRedAlliance()) {
            drive.drive(forward.getAsDouble(), strafe.getAsDouble(), turn.getAsDouble());
        } else {
            drive.drive(-forward.getAsDouble(), -strafe.getAsDouble(), turn.getAsDouble());
        }



    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // default commands run until another command requiring the subsystem interrupts them
    }
}
