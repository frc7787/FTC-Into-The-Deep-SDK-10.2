package org.firstinspires.ftc.teamcode.pedropathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedropathing.constants.LocalizerConstants;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedropathing.constants.PathFollowingConstants;


/**
 * This is the Triangle autonomous OpMode.
 * It runs the robot in a triangle, with the starting point being the bottom-middle point.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Samarth Mahapatra - 1002 CircuitRunners Robotics Surge
 * @version 1.0, 12/30/2024
 */
@Autonomous(group = "Examples")
public final class Triangle extends OpMode {
    private Follower follower;

    private final Pose startPose = new Pose(0,0, Math.toRadians(0));
    private final Pose midPose = new Pose(24, -24, Math.toRadians(90));
    private final Pose endPose = new Pose(24, 24, Math.toRadians(45));

    private PathChain triangle;

    private Telemetry multipleTelemetry;

    @Override public void init() {
        follower = new Follower(hardwareMap, PathFollowingConstants.class, LocalizerConstants.class);
        follower.setStartingPose(startPose);

        triangle = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(midPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), midPose.getHeading())
                .addPath(new BezierLine(new Point(midPose), new Point(endPose)))
                .setLinearHeadingInterpolation(midPose.getHeading(), endPose.getHeading())
                .addPath(new BezierLine(new Point(endPose), new Point(startPose)))
                .setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading())
                .build();

        follower.followPath(triangle);

        multipleTelemetry
                = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override public void loop() {
        follower.update();

        if (follower.atParametricEnd()) {
            follower.followPath(triangle, true);
        }

        follower.telemetryDebug(multipleTelemetry);
    }
}
