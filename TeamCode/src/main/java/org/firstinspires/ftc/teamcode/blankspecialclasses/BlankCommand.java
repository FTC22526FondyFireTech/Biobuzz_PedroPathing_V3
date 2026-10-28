package org.firstinspires.ftc.teamcode.blankspecialclasses;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.seattlesolvers.solverslib.command.CommandBase;

public class BlankCommand extends CommandBase {
    private TelemetryManager telemetryM;

    public BlankCommand() {

    }

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }


    @Override
    public void execute() {


    }


    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
