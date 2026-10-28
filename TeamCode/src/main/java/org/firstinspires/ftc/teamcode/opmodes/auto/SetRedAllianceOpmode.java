package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.utils.GlobalData;

@Autonomous(name = "Set Red Alliance", group = "Auto")
//@TeleOp(name = "Set Red Allianced")
//@Disabled

public class SetRedAllianceOpmode extends CommandOpMode {

    TelemetryManager telemetryM;

    @Override
    public void initialize() {

        GlobalData.setRedAlliance();

        GlobalData.allianceIsConfirmed = GlobalData.isRedAlliance();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryM.addData("Alliance is Red", GlobalData.isRedAlliance());

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