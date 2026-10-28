package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsysytem;
import org.firstinspires.ftc.teamcode.utils.PipelineConfigurables;

/**
 * Tunes and displays the BiobuzzBalls.py Limelight pipeline.
 * HSV ranges are edited live in the Panels Configurables tab (PipelineConfigurables)
 * and sent to the Limelight every loop.
 */
@TeleOp(name = "Ball Detect", group = "Test")

public class BallDetectOpmode extends CommandOpMode {

    private TelemetryManager telemetryM;
    private VisionSubsysytem vss;

    @Override
    public void initialize() {
        vss = new VisionSubsysytem(this);
        vss.setPipeline(PipelineConfigurables.BALL_PIPELINE);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void run() {
        super.run();

        vss.updatePythonInputs(PipelineConfigurables.toPythonInputs());
        double[] out = vss.getPythonOutput(PipelineConfigurables.OUT_LENGTH);
        telemetryM.addData("OUTLENGTH", out.length);

        addColor("Red nectar", out, PipelineConfigurables.OUT_RED);
        addColor("Blue nectar", out, PipelineConfigurables.OUT_BLUE);
        addColor("Yellow pollen", out, PipelineConfigurables.OUT_YELLOW);
        telemetryM.addData("HSV from robot", out[PipelineConfigurables.OUT_FROM_ROBOT] == 1.0);
        telemetryM.addData("Pipeline", vss.getPipelineNumber());
        telemetryM.update(telemetry);
    }

    private void addColor(String label, double[] out, int index) {
        int count = (int) out[index];
        if (count == 0) {
            telemetryM.addLine(label + ": none");
        } else {
            telemetryM.addLine(String.format("%s: %d  nearest %.1f in  %.1f deg",
                    label, count, out[index + 1], out[index + 2]));
        }
    }
}
