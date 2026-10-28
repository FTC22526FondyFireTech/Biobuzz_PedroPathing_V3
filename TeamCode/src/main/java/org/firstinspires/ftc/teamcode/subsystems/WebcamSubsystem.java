package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CV.BiobuzzBallPipeline_1;
import org.firstinspires.ftc.teamcode.CV.BiobuzzBallPipeline_1.Ball;
import org.firstinspires.ftc.teamcode.CV.BiobuzzBallPipeline_1.BallColor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class WebcamSubsystem extends SubsystemBase {
    public final OpenCvWebcam webcam;
    private final BiobuzzBallPipeline_1 pipeline = new BiobuzzBallPipeline_1();

    public WebcamSubsystem(HardwareMap hw) {
        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hw.get(WebcamName.class, "Webcam_1"));
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override public void onError(int errorCode) { }
        });
    }

    /** Nearest ball of any color, or null. */
    public Ball getNearest() {
        return pipeline.getLatest().nearest;
    }

    /** Nearest ball of one color, or null. */
    public Ball getNearest(BallColor color) {
        return pipeline.getLatest().nearestByColor[color.ordinal()];
    }

    public int getCount(BallColor color) {
        return pipeline.getLatest().counts[color.ordinal()];
    }

    public void stop() {
        webcam.stopStreaming();
    }
}
