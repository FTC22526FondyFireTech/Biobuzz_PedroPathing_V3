package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.ArrayDeque;
import java.util.Iterator;

/**
 * HiveMotionTracker
 * -----------------
 * Watches the vertical angle (ty) of one AprilTag on the BIOBUZZ hive and reports
 * how fast it is changing, in degrees per second, plus a simple RISING / FALLING / STILL state.
 * <p>
 * How it works (student version):
 * 1. Every loop we grab the newest Limelight result.
 * 2. If it's a NEW camera frame and our tag is in it, we save (time, ty).
 * 3. We keep only the last WINDOW_SEC seconds of samples.
 * 4. We fit a straight line through those samples. The slope of that line = rate of change.
 * (A line fit is much less jumpy than "(ty now - ty before) / dt".)
 * 5. Hysteresis thresholds turn the slope into RISING / FALLING / STILL without flickering.
 * <p>
 * IMPORTANT: ty also changes if the ROBOT moves (driving, turning, tilting). Use this while the
 * robot is parked, or only trust it when your drivetrain says you're not moving.
 * <p>
 * Sign convention: Limelight ty is POSITIVE when the target is ABOVE the crosshair.
 * So with a still robot: hive going up -> ty increasing -> positive rate -> RISING.
 * (If your camera is mounted upside-down, flip the sign with INVERT.)
 */
public class HiveMotionTracker {
    public enum Motion {RISING, FALLING, STILL, NO_TARGET}


    // ---------------- Tuning knobs ----------------
    /** AprilTag ID on the hive. Set to -1 to use whatever the Limelight's primary target is. */
    //  private final int tagId;
    /**
     * How many seconds of history to fit the line through. Bigger = smoother but slower to react.
     */
    public double WINDOW_SEC = 0.30;
    /**
     * Need at least this many samples in the window before we report a rate.
     */
    public int MIN_SAMPLES = 4;
    /**
     * Start calling it moving above this rate (deg/s).
     */
    public double ENTER_THRESHOLD_DEG_PER_SEC = 3.0;
    /**
     * Go back to STILL once the rate drops below this (deg/s). Must be < ENTER threshold.
     */
    public double EXIT_THRESHOLD_DEG_PER_SEC = 1.5;
    /**
     * If we haven't seen the tag for this long, report NO_TARGET and clear history.
     */
    public double LOST_TIMEOUT_SEC = 0.25;
    /**
     * Flip this if your camera is mounted so "up" shows as negative ty.
     */
    public boolean INVERT = false;

    // ---------------- Internal state ----------------
    private static class Sample {
        final double t;   // seconds
        final double ty;  // degrees

        Sample(double t, double ty) {
            this.t = t;
            this.ty = ty;
        }
    }

    private final ArrayDeque<Sample> samples = new ArrayDeque<>();
    private double lastFrameTime = Double.NaN;
    private double lastSeenTime = Double.NaN;
    private double rateDegPerSec = 0.0;
    private Motion motion = Motion.NO_TARGET;

    public HiveMotionTracker() {
        // this.tagId = tagId;
    }

    /**
     * Call this once every loop with limelight.getLatestResult().
     */
    public void update(LLResult result) {
        double now = System.nanoTime() / 1e9;

        Double ty = extractTy(result);
        if (ty != null) {
            // When was this frame actually captured? now minus how old the data is.
            double frameTime = now - result.getStaleness() / 1000.0;

            // Only add NEW frames. The control loop usually runs faster than the camera,
            // so we often get the same frame twice; adding it again would fake a "zero" rate.
            if (Double.isNaN(lastFrameTime) || frameTime - lastFrameTime > 0.002) {
                samples.addLast(new Sample(frameTime, INVERT ? -ty : ty));
                lastFrameTime = frameTime;
            }
            lastSeenTime = now;
        }

        // Throw away samples older than the window.
        Sample oldest = samples.peekFirst();
        while (oldest != null && now - oldest.t > WINDOW_SEC) {
            samples.removeFirst();
            oldest = samples.peekFirst();
        }

        // Lost the tag for too long? Reset.
        if (Double.isNaN(lastSeenTime) || now - lastSeenTime > LOST_TIMEOUT_SEC) {
            samples.clear();
            rateDegPerSec = 0.0;
            motion = Motion.NO_TARGET;
            return;
        }

        if (samples.size() < MIN_SAMPLES) {
            return; // keep previous answer until we have enough data
        }

        rateDegPerSec = leastSquaresSlope();
        motion = classify(rateDegPerSec, motion);
    }

    /**
     * Pull ty for our tag out of the result, or null if it isn't visible.
     */
    private Double extractTy(LLResult result) {
        if (result == null || !result.isValid()) return null;

//        if (tagId < 0) {
//            return result.getTy();
//        }
        for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
            //if (f.getFiducialId() == tagId) {
            return f.getTargetYDegrees();
            //}
        }
        return null;
    }

    /**
     * Slope of the best-fit line through (t, ty). Units: degrees per second.
     */
    private double leastSquaresSlope() {
        Sample first = samples.peekFirst();
        if (first == null) return 0.0;
        int n = samples.size();
        double t0 = first.t; // shift times so numbers stay small (better precision)
        double sumT = 0, sumY = 0, sumTT = 0, sumTY = 0;

        Iterator<Sample> it = samples.iterator();
        while (it.hasNext()) {
            Sample s = it.next();
            double t = s.t - t0;
            sumT += t;
            sumY += s.ty;
            sumTT += t * t;
            sumTY += t * s.ty;
        }
        double denom = n * sumTT - sumT * sumT;
        if (Math.abs(denom) < 1e-9) return 0.0;
        return (n * sumTY - sumT * sumY) / denom;
    }

    /**
     * Hysteresis: harder to start "moving" than to keep "moving", so it doesn't flicker.
     */
    private Motion classify(double rate, Motion previous) {
        boolean wasMoving = previous == Motion.RISING || previous == Motion.FALLING;
        double threshold = wasMoving ? EXIT_THRESHOLD_DEG_PER_SEC : ENTER_THRESHOLD_DEG_PER_SEC;

        if (rate > threshold) return Motion.RISING;
        if (rate < -threshold) return Motion.FALLING;
        return Motion.STILL;
    }

    // ---------------- Getters ----------------
    public double getRateDegPerSec() {
        return rateDegPerSec;
    }

    public Motion getMotion() {
        return motion;
    }

    public int getSampleCount() {
        return samples.size();
    }

    /**
     * Latest filtered-in ty value, or NaN if none.
     */
    public double getLatestTy() {
        Sample last = samples.peekLast();
        return last == null ? Double.NaN : last.ty;
    }

    public void reset() {
        samples.clear();
        lastFrameTime = Double.NaN;
        lastSeenTime = Double.NaN;
        rateDegPerSec = 0.0;
        motion = Motion.NO_TARGET;
    }
}
/*
 * ---------------- Example TeleOp ----------------
 *
 * @TeleOp(name = "Hive Motion Test")
 * public class HiveMotionTest extends LinearOpMode {
 *     @Override
 *     public void runOpMode() {
 *         Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
 *         limelight.setPollRateHz(100);
 *         limelight.pipelineSwitch(0);          // your AprilTag pipeline
 *         limelight.start();
 *
 *         HiveMotionTracker hive = new HiveMotionTracker(HIVE_TAG_ID); // put the real tag ID here
 *
 *         waitForStart();
 *         while (opModeIsActive()) {
 *             hive.update(limelight.getLatestResult());
 *
 *             telemetry.addData("Hive", hive.getMotion());
 *             telemetry.addData("ty (deg)", "%.2f", hive.getLatestTy());
 *             telemetry.addData("rate (deg/s)", "%.2f", hive.getRateDegPerSec());
 *             telemetry.addData("samples", hive.getSampleCount());
 *             telemetry.update();
 *         }
 *         limelight.stop();
 *     }
 * }
 */

