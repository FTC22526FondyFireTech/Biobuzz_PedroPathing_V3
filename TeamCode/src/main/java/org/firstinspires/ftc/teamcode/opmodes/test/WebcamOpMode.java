package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.CV.BiobuzzBallPipeline_1;
import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;

//@Autonomous(name = "Blank")
@TeleOp(name = "WebcamOpMode")

public class WebcamOpMode extends CommandOpMode {

    TelemetryManager telemetryM;
    GamepadEx driverGamepad;

    PanelsCameraStream pcs;

    WebcamSubsystem vss;

    @Override
    public void initialize() {

        vss = new WebcamSubsystem(this.hardwareMap);

        pcs = PanelsCameraStream.INSTANCE;

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryM.update(telemetry);

        driverGamepad = new GamepadEx(gamepad1);

        //  driverGamepad.getGamepadButton(GamepadKeys.Button.A)


        //  driverGamepad.getGamepadButton(GamepadKeys.Button.X)


    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        pcs.startStream(vss.webcam, 20);
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            run();

            telemetryM.addData("BlueCount", vss.getCount(BiobuzzBallPipeline_1.BallColor.BLUE));
            telemetryM.addData("RedCount", vss.getCount(BiobuzzBallPipeline_1.BallColor.RED));
            telemetryM.addData("YellowCount", vss.getCount(BiobuzzBallPipeline_1.BallColor.YELLOW));

            BiobuzzBallPipeline_1.Ball nearest = vss.getNearest();
            if (nearest != null) {
                telemetryM.addData("Nearest color", nearest.color);
                telemetryM.addData("Distance (in)", nearest.distanceIn);
                telemetryM.addData("Angle (deg)", nearest.angleDeg);
            } else {
                telemetry.addData("Nearest", "none seen");
            }

            BiobuzzBallPipeline_1.Ball blueBall = vss.getNearest(BiobuzzBallPipeline_1.BallColor.BLUE);

            if (blueBall != null) {
                telemetryM.addData("NearestBlueDist", blueBall.distanceIn);
                telemetryM.addData("NearestBlueAngle", blueBall.angleDeg);
            } else {
                telemetry.addData("Blue Ball", "none seen");
            }

//
//            telemetryM.addData("NearestBlue",vss.getNearest(BiobuzzBallPipeline_1.BallColor.BLUE));
//            telemetryM.addData("NearestRed",vss.getNearest(BiobuzzBallPipeline_1.BallColor.RED));
//            telemetryM.addData("NearestYellow",vss.getNearest(BiobuzzBallPipeline_1.BallColor.YELLOW));


            telemetryM.update(telemetry);
        }
        reset();
        pcs.stopStream();
    }

}