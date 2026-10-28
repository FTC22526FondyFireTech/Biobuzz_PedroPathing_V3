package org.firstinspires.ftc.teamcode.CV;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * Biobuzz game piece detection - EasyOpenCV pipeline with EOCV-Sim / VisionBench Variable Tuner support.
 * Port of BiobuzzBalls.py (Limelight Snapscript).
 *
 * TUNING: every public, non-final field below shows up in the Variable Tuner.
 *   - redLower / redUpper, blueLower / blueUpper, yellowLower / yellowUpper are HSV Scalars
 *     (H 0-180, S 0-255, V 0-255). The tuner sliders go to 255, but hue stops at 180.
 *   - Red wraps through 0: if lower H > upper H (e.g. 165 .. 10) both ends of the hue circle are used.
 *   - Set "view" to RED_MASK / BLUE_MASK / YELLOW_MASK to see only what that color's range picks up.
 *   - When happy, copy the numbers back into the defaults below.
 *
 * Outputs (getOutputs()) use the same layout as the Python llpython array:
 *   [0] red count    [1] red dist    [2] red angle
 *   [3] blue count   [4] blue dist   [5] blue angle
 *   [6] yellow count [7] yellow dist [8] yellow angle
 *   [9] 1 if HSV ranges were set from robot code, 0 otherwise
 *   dist  = inches along the floor from the camera to the ball (0 if none seen)
 *   angle = degrees from the camera's forward direction, + = ball to the right
 *
 * EasyOpenCV frames are RGBA, so the pipeline converts RGBA -> RGB -> HSV.
 */
public class BiobuzzBallPipeline_1 extends OpenCvPipeline {

    // ============================================================ TUNABLE (Variable Tuner)

    /** What the preview shows. */
    public enum ViewMode { ANNOTATED, RED_MASK, BLUE_MASK, YELLOW_MASK }
    public ViewMode view = ViewMode.ANNOTATED;

    // HSV thresholds (H, S, V) - red wraps because lower H > upper H
    public Scalar redLower    = new Scalar(165, 130, 60);
    public Scalar redUpper    = new Scalar(10, 255, 255);
    public Scalar blueLower   = new Scalar(100, 140, 50);
    public Scalar blueUpper   = new Scalar(125, 255, 255);
    public Scalar yellowLower = new Scalar(18, 150, 151);
    public Scalar yellowUpper = new Scalar(35, 255, 255);

    // Contour filters (pixels at 640x480)
    public double minArea = 150;
    public double maxArea = 40000;
    public double minCircularity = 0.40;

    // Geometry - keep in sync with PipelineConfigurables.java
    public double cameraHeightIn = 12.0;
    public double cameraPitchDownDeg = 20.0;

    // Camera field of view (Limelight 3A values - change these for your webcam)
    double hfovDeg = 53.0;
   double vfovDeg = 31.0;

    // ============================================================ constants / types

    static final double NECTAR_DIAMETER_IN = 3.6;
    static final double POLLEN_DIAMETER_IN = 2.8;

    public enum BallColor {
        // draw colors are RGBA because the EasyOpenCV frame is RGBA
        RED(NECTAR_DIAMETER_IN, new Scalar(255, 0, 0, 255)),
        BLUE(NECTAR_DIAMETER_IN, new Scalar(0, 0, 255, 255)),
        YELLOW(POLLEN_DIAMETER_IN, new Scalar(255, 255, 0, 255));

        public final double diameterIn;
        public final Scalar drawColor;

        BallColor(double diameterIn, Scalar drawColor) {
            this.diameterIn = diameterIn;
            this.drawColor = drawColor;
        }
    }

    /** One detected ball. */
    public static class Ball {
        public final BallColor color;
        public final double distanceIn;
        public final double angleDeg;
        public final Rect box;

        Ball(BallColor color, double distanceIn, double angleDeg, Rect box) {
            this.color = color;
            this.distanceIn = distanceIn;
            this.angleDeg = angleDeg;
            this.box = box;
        }
    }

    /** Results of one frame. */
    public static class FrameResult {
        public final int[] counts = new int[BallColor.values().length];
        public final Ball[] nearestByColor = new Ball[BallColor.values().length];
        public Ball nearest;          // nearest ball of any color
        public boolean rangesFromRobot;
    }

    // ============================================================ private state

    private volatile boolean rangesFromRobot = false;
    private volatile FrameResult latest = new FrameResult();

    private final Mat rgb = new Mat();
    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();
    private final Mat m1 = new Mat();
    private final Mat m2 = new Mat();
    private final Mat hierarchy = new Mat();
    private final Mat maskView = new Mat();
    private final Mat kernelClose = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(7, 7));
    private final Mat kernelOpen = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));

    // ============================================================ inputs from robot code

    /** Override the HSV range for one color from OpMode code. */
    public synchronized void setRange(BallColor color, Scalar lower, Scalar upper) {
        switch (color) {
            case RED:    redLower = lower;    redUpper = upper;    break;
            case BLUE:   blueLower = lower;   blueUpper = upper;   break;
            case YELLOW: yellowLower = lower; yellowUpper = upper; break;
        }
        rangesFromRobot = true;
    }

    /**
     * Same packed input array the Python script read from llrobot (each value = H*65536 + S*256 + V):
     *   [0] red low  [1] red high  [2] blue low  [3] blue high  [4] yellow low  [5] yellow high
     * A pair of zeros leaves that color unchanged.
     */
    public synchronized void setPackedInputs(double[] packed) {
        BallColor[] colors = BallColor.values();
        for (int i = 0; i < colors.length; i++) {
            int lo = (int) packed[i * 2];
            int hi = (int) packed[i * 2 + 1];
            if (lo == 0 && hi == 0) continue;
            setRange(colors[i], unpack(lo), unpack(hi));
        }
    }

    private static Scalar unpack(int v) {
        return new Scalar((v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF);
    }

    // ============================================================ outputs

    /** Latest results (safe to call from the OpMode thread). */
    public FrameResult getLatest() {
        return latest;
    }

    /** Latest results in the same 10-value layout as the Python llpython array. */
    public double[] getOutputs() {
        FrameResult r = latest;
        double[] out = new double[10];
        for (BallColor c : BallColor.values()) {
            int i = c.ordinal();
            Ball b = r.nearestByColor[i];
            out[i * 3] = r.counts[i];
            out[i * 3 + 1] = b == null ? 0.0 : b.distanceIn;
            out[i * 3 + 2] = b == null ? 0.0 : b.angleDeg;
        }
        out[9] = r.rangesFromRobot ? 1.0 : 0.0;
        return out;
    }

    // ============================================================ pipeline

    @Override
    public Mat processFrame(Mat input) {
        if (input.empty()) return input;

        int w = input.cols();
        int h = input.rows();

        // EasyOpenCV gives RGBA (4 channels): RGBA -> RGB -> HSV
        Imgproc.cvtColor(input, rgb, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(rgb, hsv, Imgproc.COLOR_RGB2HSV);

        // Snapshot the tunable values once per frame (the tuner can change them at any time)
        Scalar[][] ranges;
        FrameResult result = new FrameResult();
        synchronized (this) {
            ranges = new Scalar[][] {
                    { redLower, redUpper },
                    { blueLower, blueUpper },
                    { yellowLower, yellowUpper },
            };
            result.rangesFromRobot = rangesFromRobot;
        }
        ViewMode mode = view;
        Mat maskToShow = null;

        for (BallColor color : BallColor.values()) {
            int i = color.ordinal();
            makeMask(ranges[i][0], ranges[i][1]);

            if (mode.ordinal() == i + 1) {      // RED_MASK / BLUE_MASK / YELLOW_MASK
                maskView.release();
                Core.bitwise_and(input, input, maskView, mask);
                maskToShow = maskView;
            }

            List<Ball> balls = findBalls(input, color, w, h);   // also draws outlines on input
            result.counts[i] = balls.size();

            Ball best = null;
            for (Ball b : balls) {
                if (best == null || b.distanceIn < best.distanceIn) best = b;
            }
            result.nearestByColor[i] = best;

            if (best != null) {
                String label = String.format("%.0fin %.0fdeg", best.distanceIn, best.angleDeg);
                Imgproc.putText(input, label,
                        new Point(best.box.x, Math.max(12, best.box.y - 6)),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, color.drawColor, 2);
                if (result.nearest == null || best.distanceIn < result.nearest.distanceIn) {
                    result.nearest = best;
                }
            }
        }

        latest = result;
        return maskToShow != null ? maskToShow : input;
    }

    private void makeMask(Scalar lower, Scalar upper) {
        double lh = lower.val[0], ls = lower.val[1], lv = lower.val[2];
        double hh = upper.val[0], hs = upper.val[1], hv = upper.val[2];
        if (lh <= hh) {
            Core.inRange(hsv, new Scalar(lh, ls, lv), new Scalar(hh, hs, hv), mask);
        } else {
            // hue wraps through 0 (red)
            Core.inRange(hsv, new Scalar(lh, ls, lv), new Scalar(180, hs, hv), m1);
            Core.inRange(hsv, new Scalar(0, ls, lv), new Scalar(hh, hs, hv), m2);
            Core.bitwise_or(m1, m2, mask);
        }
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernelClose);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernelOpen);
    }

    private List<Ball> findBalls(Mat drawOn, BallColor color, int w, int h) {
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        List<Ball> balls = new ArrayList<>();
        List<MatOfPoint> keep = new ArrayList<>();
        for (MatOfPoint cnt : contours) {
            double area = Imgproc.contourArea(cnt);
            if (area < minArea || area > maxArea) continue;

            MatOfPoint2f cnt2f = new MatOfPoint2f(cnt.toArray());
            double perim = Imgproc.arcLength(cnt2f, true);
            cnt2f.release();
            if (perim < 10) continue;
            if (4 * Math.PI * area / (perim * perim) < minCircularity) continue;

            Rect box = Imgproc.boundingRect(cnt);
            double[] pos = locate(box.x + box.width / 2.0, box.y + box.height / 2.0, w, h, color.diameterIn);
            if (pos == null) continue;

            balls.add(new Ball(color, pos[0], pos[1], box));
            keep.add(cnt);
        }

        if (!keep.isEmpty()) {
            Imgproc.drawContours(drawOn, keep, -1, color.drawColor, 2);
        }
        for (MatOfPoint cnt : contours) cnt.release();
        return balls;
    }

    /** Ground distance (in) and bearing (deg) of a ball centre at pixel (cx, cy), or null. */
    private double[] locate(double cx, double cy, int w, int h, double diameter) {
        double fx = (w / 2.0) / Math.tan(Math.toRadians(hfovDeg / 2.0));
        double fy = (h / 2.0) / Math.tan(Math.toRadians(vfovDeg / 2.0));
        double x = (cx - w / 2.0) / fx;      // right of centre
        double y = (cy - h / 2.0) / fy;      // below centre
        double pitch = Math.toRadians(cameraPitchDownDeg);

        // Ray in floor frame: forward = cos(p) - y*sin(p), down = sin(p) + y*cos(p)
        double forward = Math.cos(pitch) - y * Math.sin(pitch);
        double down = Math.sin(pitch) + y * Math.cos(pitch);
        if (down <= 0.01 || forward <= 0) return null;  // ray never reaches the ball's height

        double t = (cameraHeightIn - diameter / 2.0) / down;  // ball centre sits diameter/2 above floor
        double groundForward = t * forward;
        double lateral = t * x;
        return new double[] {
                Math.hypot(groundForward, lateral),
                Math.toDegrees(Math.atan2(lateral, groundForward))
        };
    }
}
