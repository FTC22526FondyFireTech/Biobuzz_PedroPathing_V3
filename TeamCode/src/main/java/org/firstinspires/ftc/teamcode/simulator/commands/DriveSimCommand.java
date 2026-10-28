package org.firstinspires.ftc.teamcode.simulator.commands;

import com.seattlesolvers.solverslib.command.CommandBase;


import org.firstinspires.ftc.teamcode.simulator.drivetrains.MecanumDriveSubsystemSimulation;
import org.firstinspires.ftc.teamcode.utils.GlobalData;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Default teleop drive command. Continuously feeds gamepad stick input to the
 * {@link MecanumDriveSubsystemSimulation} until interrupted (e.g. by an auto-align command
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
public class DriveSimCommand extends CommandBase {

    private final MecanumDriveSubsystemSimulation driveSim;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier turn;
    private final BooleanSupplier fieldCentric;

    public DriveSimCommand(MecanumDriveSubsystemSimulation driveSim,
                           DoubleSupplier forward,
                           DoubleSupplier strafe,
                           DoubleSupplier turn,
                           BooleanSupplier fieldCentric
    ) {
        this.driveSim = driveSim;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        this.fieldCentric = fieldCentric;
        addRequirements(driveSim);
    }

    @Override
    public void initialize() {
        driveSim.startTeleopDrive();
    }

    @Override
    public void execute() {
        if (GlobalData.isRedAlliance())
            driveSim.drive(forward.getAsDouble(), strafe.getAsDouble(), turn.getAsDouble());
        else
            driveSim.drive(-forward.getAsDouble(), -strafe.getAsDouble(), turn.getAsDouble());

    }

    @Override
    public void end(boolean interrupted) {
        driveSim.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // default commands run until another command requiring the subsystem interrupts them
    }
}
