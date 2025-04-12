package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedropathing.constants.LocalizerConstants;
import org.firstinspires.ftc.teamcode.pedropathing.constants.PathFollowingConstants;

@Autonomous(group = "$")
public class Bucket extends OpMode {
    private Follower follower;

    private Timer pathTimer,
                  actionTimer,
                  opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(8.0, 103.0, Math.toRadians(90));

    private final Pose bucketPose = new Pose(16.0, 125.0, Math.toRadians(135));

    private final Pose barPose = new Pose(41, 70, Math.toRadians(180.0));

    private final Pose spikeOneControlPointOne = new Pose(0.0, 24.5);

    private final Pose spikeOneControlPointTwo = new Pose(57.0, 45.0);

    private final Pose spikeOnePose = new Pose(12.0, 121.5, Math.toRadians(1));

    private final Pose nearWallPose = new Pose(16.0, 30.0, Math.toRadians(0));

    private final Pose wallPose = new Pose(8.0, 32.0, Math.toRadians(0));

    private final Pose spikeTwoControlPoint = new Pose(67.0, 35.0);

    private final Pose spikeTwoPose = new Pose(16.0, 129.5, Math.toRadians(0.1));

    private final Pose spikeTwoToWallControlPointOne = new Pose(-5.2, 10.9);

    private final Pose spikeTwoToWallControlPointTwo = new Pose(37.5, 30.8);

    private final Pose spikeThreePose = new Pose(18, 132, Math.toRadians(10));

    private final Pose parkPose = new Pose(63, 96, Math.toRadians(90));

    private Path scorePreload;
    private PathChain spikeOne,
            spikeOneToBucket,
            spikeTwo,
            spikeTwoToBucket,
            spikeThree,
            spikeThreeToBucket,
            wallToBarTwo,
            park;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(bucketPose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), bucketPose.getHeading());

        spikeOne = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(bucketPose),
                        new Point(spikeOnePose))
                )
                .setLinearHeadingInterpolation(barPose.getHeading(), spikeOnePose.getHeading())
                .build();

        spikeOneToBucket = follower.pathBuilder()
                .addPath(new BezierLine(new Point(spikeOnePose), new Point(bucketPose)))
                .setLinearHeadingInterpolation(spikeOnePose.getHeading(), bucketPose.getHeading())
                .build();

        spikeTwo = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(bucketPose),
                        new Point(spikeTwoPose)
                ))
                .setLinearHeadingInterpolation(bucketPose.getHeading(), spikeTwoPose.getHeading())
                .build();

        spikeTwoToBucket = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(spikeTwoPose),
                        new Point(bucketPose)
                ))
                .setLinearHeadingInterpolation(spikeTwoPose.getHeading(), bucketPose.getHeading())
                .build();

        spikeThree = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bucketPose), new Point(spikeThreePose)))
                .setLinearHeadingInterpolation(bucketPose.getHeading(), spikeThreePose.getHeading())
                .build();

        spikeThreeToBucket = follower.pathBuilder()
                .addPath(new BezierLine(new Point(spikeThreePose), new Point(bucketPose)))
                .setLinearHeadingInterpolation(spikeThreePose.getHeading(), bucketPose.getHeading())
                .build();

        wallToBarTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(wallPose), new Point(barPose)))
                .setLinearHeadingInterpolation(wallPose.getHeading(), barPose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(bucketPose),
                        new Point(64, 127),
                        new Point(parkPose)))
                .setLinearHeadingInterpolation(bucketPose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

                if(!follower.isBusy()) {
                    follower.followPath(spikeOne, true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(spikeOneToBucket, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(spikeTwo, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(spikeTwoToBucket,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(spikeThree,true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(spikeThreeToBucket, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(park);
                    setPathState(8);
                }
                break;
            case 8:
                if (follower.isRobotStuck()) follower.breakFollowing();
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override public void loop() {
        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(
                hardwareMap,
                PathFollowingConstants.class,
                LocalizerConstants.class
        );
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}
