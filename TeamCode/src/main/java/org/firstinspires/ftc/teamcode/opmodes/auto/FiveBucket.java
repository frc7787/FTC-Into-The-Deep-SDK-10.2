package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.Point;

public class FiveBucket {

    public FiveBucket() {
        PathBuilder builder = new PathBuilder();

        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(7.898, 113.463, Point.CARTESIAN),
                                new Point(30.801, 120.044, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(30.801, 120.044, Point.CARTESIAN),
                                new Point(13.163, 129.784, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(13.163, 129.784, Point.CARTESIAN),
                                new Point(32.380, 130.837, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(32.380, 130.837, Point.CARTESIAN),
                                new Point(13.426, 129.521, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(13.426, 129.521, Point.CARTESIAN),
                                new Point(33.697, 134.523, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(30))
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(33.697, 134.523, Point.CARTESIAN),
                                new Point(13.163, 129.521, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(135))
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(13.163, 129.521, Point.CARTESIAN),
                                new Point(11.583, 36.329, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(270))
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(11.583, 36.329, Point.CARTESIAN),
                                new Point(13.163, 129.521, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(135))
                .addPath(
                        // Line 9
                        new BezierCurve(
                                new Point(13.163, 129.521, Point.CARTESIAN),
                                new Point(25.009, 113.199, Point.CARTESIAN),
                                new Point(63.181, 124.519, Point.CARTESIAN),
                                new Point(61.865, 95.298, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true);
    }
}

