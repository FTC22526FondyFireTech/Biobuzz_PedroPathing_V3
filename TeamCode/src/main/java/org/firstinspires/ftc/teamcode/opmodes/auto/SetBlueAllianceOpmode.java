package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.utils.GlobalData;

@Autonomous(name = "Set Blue Alliance", group = "Auto")
//@TeleOp(name = "Set Blue Allianced")
//@Disabled

public class SetBlueAllianceOpmode extends CommandOpMode {

    TelemetryManager telemetryM;

    @Override
    public void initialize() {

        GlobalData.setBlueAlliance();

        GlobalData.allianceIsConfirmed = GlobalData.isBlueAlliance();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryM.addData("Alliance is Blue", GlobalData.isBlueAlliance());

        telemetryM.addData("Alliance Confirmed", GlobalData.allianceIsConfirmed);

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