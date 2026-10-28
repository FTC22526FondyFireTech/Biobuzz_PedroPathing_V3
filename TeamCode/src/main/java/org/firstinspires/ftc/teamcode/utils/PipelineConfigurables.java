package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Tunable values for the Limelight 3A Snapscript ball-detection pipeline.
 * <p>
 * The pipeline runs on the Limelight, so these values reach it through
 * limelight.updatePythonInputs(double[8]) - see {@link #toPythonInputs()}.
 * <p>
 * HSV uses OpenCV ranges: H 0-180, S 0-255, V 0-255.
 * If a color's low H is greater than its high H the hue range wraps through 0
 * (used for red, e.g. 165 -> 10).
 */
@Configurable
public class PipelineConfigurables {

    // Fixed geometry and game piece sizes (not tunable from the dashboard)
    public static final double CAMERA_HEIGHT_IN = 12.0;
    public static final double CAMERA_PITCH_DOWN_DEG = 20.0;
    public static final double NECTAR_DIAMETER_IN = 3.6;   // red and blue
    public static final double POLLEN_DIAMETER_IN = 2.8;   // yellow

    // Limelight pipeline slot that holds BiobuzzBalls.py
    public static final int BALL_PIPELINE = 5;

    // Pipeline output layout (llpython), 3 values per color then a flag
    public static final int OUT_RED = 0;      // count, distance in, angle deg
    public static final int OUT_BLUE = 3;
    public static final int OUT_YELLOW = 6;
    public static final int OUT_FROM_ROBOT = 9;
    public static final int OUT_LENGTH = 10;

    // Red nectar (wraps around hue 0)
    public static int redLowH = 165;
    public static int redLowS = 130;
    public static int redLowV = 60;
    public static int redHighH = 10;
    public static int redHighS = 255;
    public static int redHighV = 255;

    // Blue nectar
    public static int blueLowH = 100;
    public static int blueLowS = 140;
    public static int blueLowV = 50;
    public static int blueHighH = 125;
    public static int blueHighS = 255;
    public static int blueHighV = 255;

    // Yellow pollen
    public static int yellowLowH = 18;
    public static int yellowLowS = 90;
    public static int yellowLowV = 80;
    public static int yellowHighH = 35;
    public static int yellowHighS = 255;
    public static int yellowHighV = 255;

    /** Pack one HSV triple into a single exactly-representable double. */
    private static double pack(int h, int s, int v) {
        return clamp(h, 180) * 65536.0 + clamp(s, 255) * 256.0 + clamp(v, 255);
    }

    private static int clamp(int value, int max) {
        return Math.max(0, Math.min(max, value));
    }

    /**
     * The Limelight accepts only 8 doubles, so each HSV bound is packed into one:
     * value = H * 65536 + S * 256 + V. The pipeline unpacks them.
     * <p>
     * [0] red low, [1] red high, [2] blue low, [3] blue high,
     * [4] yellow low, [5] yellow high, [6] and [7] reserved (0).
     */
    public static double[] toPythonInputs() {
        return new double[]{
                pack(redLowH, redLowS, redLowV),
                pack(redHighH, redHighS, redHighV),
                pack(blueLowH, blueLowS, blueLowV),
                pack(blueHighH, blueHighS, blueHighV),
                pack(yellowLowH, yellowLowS, yellowLowV),
                pack(yellowHighH, yellowHighS, yellowHighV),
                0, 0
        };
    }
}
