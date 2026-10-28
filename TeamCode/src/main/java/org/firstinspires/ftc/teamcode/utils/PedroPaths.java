package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Small helpers for building Pedro Pathing 3 {@link Path}s. These replace the Pedro 2
 * {@code follower.pathBuilder().addPath(new BezierLine/BezierCurve(...)).build()} pattern.
 * <p>
 * The "linear" variants interpolate heading from the first pose's heading to the last pose's
 * heading (Pedro 2's {@code setLinearHeadingInterpolation(start.heading(), end.heading())});
 * the "tangent" variants face along the path (Pedro 2's default for a path with no heading
 * interpolation set). Intermediate poses of a curve are control points: only their x/y matter.
 */
public final class PedroPaths {

    private PedroPaths() {
    }

    public static Path line(Pose start, Pose end) {
        return Paths.line(start, end).linear(start, end);
    }

    public static Path lineTangent(Pose start, Pose end) {
        return Paths.line(start, end).tangent();
    }

    /** Bezier curve through {@code points} (start, control points..., end) with linear heading. */
    public static Path curve(Pose... points) {
        return Paths.curve(points).linear(points[0], points[points.length - 1]);
    }

    /** Bezier curve through {@code points} (start, control points..., end) facing along the path. */
    public static Path curveTangent(Pose... points) {
        return Paths.curve(points).tangent();
    }
}
