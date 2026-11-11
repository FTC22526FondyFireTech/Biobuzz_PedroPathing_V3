package org.firstinspires.ftc.teamcode.utils;


import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Wrapper for the REV Color Sensor V3 (Broadcom APDS-9151).
 *
 * Robot Configuration: add an I2C device of type "REV Color Sensor V3"
 * and name it to match the string passed to the constructor (e.g. "color").
 *
 * Two ways to identify a color:
 *   1. getDetectedColor()  - simple hue-range classification.
 *   2. matchColor()        - matches the reading to a list of preset colors
 *                            (closest match wins, with a confidence score),
 *                            similar to REV's ColorMatch class for FRC.
 */
public class ColorMatch {

    /** Simple hue-based color categories. */
    public enum DetectedColor { RED, YELLOW, GREEN, BLUE, PURPLE, WHITE, UNKNOWN, NONE }

    // ---- Tuning values ----
    public static float GAIN = 8.0f;                 // Higher = brighter readings. 2-15 is typical.
    public static double DETECT_DISTANCE_CM = 4.0;   // Object must be closer than this to classify/match.
    public static float MIN_SATURATION = 0.30f;      // Below this, hue classifier reports WHITE.
    public static double MIN_CONFIDENCE = 0.95;      // Preset match must be at least this confident.

    // =====================================================================
    //  Preset color matching
    // =====================================================================

    /** A named reference color, stored as a unit vector so brightness doesn't matter. */
    public static class ColorPreset {
        public final String name;
        public final double r, g, b;

        public ColorPreset(String name, double red, double green, double blue) {
            this.name = name;
            double mag = Math.sqrt(red * red + green * green + blue * blue);
            if (mag == 0) mag = 1;
            this.r = red / mag;
            this.g = green / mag;
            this.b = blue / mag;
        }

        /** Java line you can paste into your code to recreate this preset. */
        public String toCode() {
            return String.format(Locale.US, "addPreset(\"%s\", %.3f, %.3f, %.3f);", name, r, g, b);
        }
    }

    /** Result of a preset match. preset is null if nothing matched. */
    public static class MatchResult {
        public final ColorPreset preset;      // best match that passed MIN_CONFIDENCE, or null
        public final ColorPreset closest;     // closest preset, even if below the threshold
        public final double confidence;       // 0..1, 1 = perfect match

        MatchResult(ColorPreset preset, ColorPreset closest, double confidence) {
            this.preset = preset;
            this.closest = closest;
            this.confidence = confidence;
        }

        public boolean matched()       { return preset != null; }
        public boolean is(String name) { return preset != null && preset.name.equals(name); }
        public String name()           { return preset == null ? "NONE" : preset.name; }
    }

    private final List<ColorPreset> presets = new ArrayList<>();

    /** Add (or replace, if the name already exists) a preset color. */
    public void addPreset(String name, double red, double green, double blue) {
        removePreset(name);
        presets.add(new ColorPreset(name, red, green, blue));
    }

    /** Save the sensor's current reading as a preset. Call update() first. */
    public ColorPreset capturePreset(String name) {
        addPreset(name, lastColors.red, lastColors.green, lastColors.blue);
        return getPreset(name);
    }

    public void removePreset(String name) {
        for (int i = presets.size() - 1; i >= 0; i--) {
            if (presets.get(i).name.equals(name)) presets.remove(i);
        }
    }

    public void clearPresets()             { presets.clear(); }
    public List<ColorPreset> getPresets()  { return presets; }

    public ColorPreset getPreset(String name) {
        for (ColorPreset p : presets) if (p.name.equals(name)) return p;
        return null;
    }

    /** Match the current reading against all presets. Call update() first. */
    public MatchResult matchColor() {
        if (presets.isEmpty() || lastColors == null) return new MatchResult(null, null, 0);

        double r = lastColors.red, g = lastColors.green, b = lastColors.blue;
        double mag = Math.sqrt(r * r + g * g + b * b);
        if (mag == 0) return new MatchResult(null, null, 0);
        r /= mag; g /= mag; b /= mag;

        ColorPreset best = null;
        double bestDist = Double.MAX_VALUE;
        for (ColorPreset p : presets) {
            double dr = r - p.r, dg = g - p.g, db = b - p.b;
            double dist = Math.sqrt(dr * dr + dg * dg + db * db);
            if (dist < bestDist) { bestDist = dist; best = p; }
        }

        double confidence = Math.max(0, 1.0 - bestDist);
        boolean ok = confidence >= MIN_CONFIDENCE && isObjectPresent();
        return new MatchResult(ok ? best : null, best, confidence);
    }

    /** Starting presets. Recalibrate these with the test OpMode for your sensor and game pieces. */
    public void loadDefaultPresets() {
        addPreset("RED",    0.561, 0.232, 0.114);
        addPreset("YELLOW", 0.361, 0.524, 0.113);
        addPreset("GREEN",  0.197, 0.561, 0.240);
        addPreset("BLUE",   0.143, 0.427, 0.429);
    }

    // =====================================================================
    //  Sensor access
    // =====================================================================

    private final NormalizedColorSensor colorSensor;
    private final DistanceSensor distanceSensor;
    private final float[] hsv = new float[3];
    private NormalizedRGBA lastColors;
    private double lastDistanceCm = Double.MAX_VALUE;

    public ColorMatch(HardwareMap hardwareMap, String deviceName) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, deviceName);
        // The V3 driver also implements DistanceSensor.
        distanceSensor = (DistanceSensor) colorSensor;
        colorSensor.setGain(GAIN);
        setLight(true);
        loadDefaultPresets();
    }

    /** Call once per loop before using the getters. */
    public void update() {
        lastColors = colorSensor.getNormalizedColors();
        Color.colorToHSV(lastColors.toColor(), hsv);
        lastDistanceCm = distanceSensor.getDistance(DistanceUnit.CM);
    }

    public void setGain(float gain) {
        GAIN = gain;
        colorSensor.setGain(gain);
    }

    public float getGain() { return colorSensor.getGain(); }

    /** Turns the onboard LED on/off if the driver supports it. */
    public void setLight(boolean on) {
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(on);
        }
    }

    public NormalizedRGBA getRGBA()   { return lastColors; }
    public float getHue()             { return hsv[0]; }  // 0-360 degrees
    public float getSaturation()      { return hsv[1]; }  // 0-1
    public float getValue()           { return hsv[2]; }  // 0-1
    public double getDistanceCm()     { return lastDistanceCm; }

    public boolean isObjectPresent() {
        return lastDistanceCm < DETECT_DISTANCE_CM;
    }

    /** Simple hue-range classification (alternative to matchColor()). */
    public DetectedColor getDetectedColor() {
        if (!isObjectPresent()) return DetectedColor.NONE;
        if (hsv[1] < MIN_SATURATION) return DetectedColor.WHITE;

        float h = hsv[0];
        if (h < 20 || h >= 330) return DetectedColor.RED;
        if (h < 70)             return DetectedColor.YELLOW;
        if (h < 160)            return DetectedColor.GREEN;
        if (h < 250)            return DetectedColor.BLUE;
        if (h < 330)            return DetectedColor.PURPLE;
        return DetectedColor.UNKNOWN;
    }
}
