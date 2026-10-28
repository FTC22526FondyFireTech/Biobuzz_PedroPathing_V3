package org.firstinspires.ftc.teamcode.blankspecialclasses;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

//@Autonomous(name = "Blank")
@TeleOp(name = "Blank")
@Disabled

public class BlankCommandOpmode extends CommandOpMode {

    TelemetryManager telemetryM;

    @Override
    public void initialize() {

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryM.update(telemetry);


    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            run();
            telemetryM.update(telemetry);
        }
        reset();
    }

}