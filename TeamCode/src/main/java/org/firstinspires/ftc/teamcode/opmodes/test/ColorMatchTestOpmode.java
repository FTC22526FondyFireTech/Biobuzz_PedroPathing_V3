package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.utils.ColorMatch;

//@Autonomous(name = "Blank")
@TeleOp(name = "Color Match")
@Disabled

public class ColorMatchTestOpmode extends CommandOpMode {

    TelemetryManager telemetryM;
    ColorMatch colorMatch;
    ColorMatch.DetectedColor detectedColor;

    ColorMatch.MatchResult matchedColor;
    boolean objectPresent;
    double distanceInch;

    boolean redDetected;
    boolean blueDetected;
    boolean yellowDetected;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void initialize() {
        colorMatch = new ColorMatch(hardwareMap, "sensor_color");

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryM.update(telemetry);




    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            run();
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.a && !previousGamepad1.a) {
                ColorMatch.ColorPreset redPreset = colorMatch.capturePreset("RED");
            }
            if (currentGamepad1.b && !previousGamepad1.b) {
                ColorMatch.ColorPreset bluePreset = colorMatch.capturePreset("BLUE");
            }
            if (currentGamepad1.y && !previousGamepad1.y) {
                ColorMatch.ColorPreset yellowPreset = colorMatch.capturePreset("YELLOW");
            }

            if (currentGamepad1.a && !previousGamepad1.y) {
               ColorMatch.ColorPreset clearPresets;
            }




            colorMatch.update();

            objectPresent = colorMatch.isObjectPresent();
            detectedColor = colorMatch.getDetectedColor();
            distanceInch = colorMatch.getDistanceInch();
            matchedColor = colorMatch.matchColor();


            telemetryM.addData("Object Present", objectPresent);
            telemetryM.addData("Distance Inch", distanceInch);

            telemetryM.addData("Match Found", matchedColor.matched());
            telemetryM.addData("Match Name", matchedColor.name());
            telemetryM.addData("Confidence", matchedColor.confidence);
            telemetryM.addData("Closest", matchedColor.closest);

            telemetryM.addData("Hue", colorMatch.getHue());
            telemetryM.addData("Saturation", colorMatch.getSaturation());
            telemetryM.addData("Value", colorMatch.getValue());
            telemetryM.update(telemetry);
            currentGamepad1.copy(gamepad1);
        }
        reset();
    }

}