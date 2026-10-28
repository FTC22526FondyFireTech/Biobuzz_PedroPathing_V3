package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Draws the robot and the active path onto the FTC Panels field view.
 * Adapted from the official Pedro Pathing quickstart's Panels Drawing class.
 * Call {@link #init()} once (e.g. in the subsystem constructor), draw each loop, then call
 * {@link #sendPacket()}.
 */
public class Drawing {
    public static final double ROBOT_RADIUS = 9;
    private static final int PATH_SAMPLES = 24;
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
    private static final Style robotLook = new Style("", "#3F51B5", 0.0);

    /** Prepares the Panels field view to use Pedro Pathing's field offsets. Call once. */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    public static void drwaLine(double x1, double y1, double x2, double y2){
        panelsField.setStyle(robotLook);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    private static boolean isValid(Pose pose) {
        return pose != null && !Double.isNaN(pose.x()) && !Double.isNaN(pose.y()) && !Double.isNaN(pose.heading());
    }

    public static void drawRobot(Pose pose, Style style) {
        if (!isValid(pose)) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.x(), pose.y());
        panelsField.circle(ROBOT_RADIUS);

        double dx = Math.cos(pose.heading()) * ROBOT_RADIUS;
        double dy = Math.sin(pose.heading()) * ROBOT_RADIUS;
        double x1 = pose.x() + dx / 2, y1 = pose.y() + dy / 2;
        double x2 = pose.x() + dx, y2 = pose.y() + dy;

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    public static void drawPose(Pose pose) {
        if (!isValid(pose)) {
            return;
        }
        Style poseLook = new Style("", "#4CBB17", 1.0);
        panelsField.setStyle(poseLook);
        panelsField.moveCursor(pose.x(), pose.y());
        panelsField.circle(5);

        double dx = Math.cos(pose.heading()) * 5;
        double dy = Math.sin(pose.heading()) * 5;
        double x1 = pose.x() + dx / 2, y1 = pose.y() + dy / 2;
        double x2 = pose.x() + dx, y2 = pose.y() + dy;

        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
        panelsField.update();
    }

    /** Draws a path as a polyline sampled along its length. */
    public static void drawPath(Path path, Style style) {
        panelsField.setStyle(style);
        Pose previous = path.get(0);
        for (int i = 1; i <= PATH_SAMPLES; i++) {
            Pose next = path.get((double) i / PATH_SAMPLES);
            if (isValid(previous) && isValid(next)) {
                panelsField.moveCursor(previous.x(), previous.y());
                panelsField.line(next.x(), next.y());
            }
            previous = next;
        }
    }

    public static void sendPacket() {
        panelsField.update();
    }
}
