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
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.hardware.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.pedropathing.constants.LocalizerConstants;
import org.firstinspires.ftc.teamcode.pedropathing.constants.PathFollowingConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Example Auto Blue", group = "Examples")
public class Bar extends OpMode {

    private static final double HIGH_BAR_EXTENSION_INCHES = 35.5;
    private static final double HIGH_BAR_ROTATION_DEGREES = 85.0;

    private Follower follower;
    private Arm arm;

    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(8, 55, Math.toRadians(90.0));

    private final Pose barPose = new Pose(41, 70, Math.toRadians(180.0));

    private final Pose spikeOneControlPointOne = new Pose(0.0, 24.5);

    private final Pose spikeOneControlPointTwo = new Pose(57.0, 45.0);

    private final Pose spikeOnePose = new Pose(58, 30, Math.toRadians(0));

    private final Pose nearWallPose = new Pose(16, 30, Math.toRadians(0));

    private final Pose wallPose = new Pose(10, 32, Math.toRadians(0));

    private final Pose spikeTwoControlPoint = new Pose(67, 35);

    private final Pose spikeTwoPose = new Pose(60.0, 16.5, Math.toRadians(0));

    private final Pose spikeTwoToWallControlPointOne = new Pose(-5.2, 10.9);

    private final Pose spikeTwoToWallControlPointTwo = new Pose(37.5, 30.8);


    private PathChain scorePreload,
                      spikeOne,
                      spikeOneToWall,
                      spikeTwo,
                      spikeTwoToWall,
                      wallToBar,
                      barToWall,
                      wallToBarTwo,
                      park;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, barPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), barPose.getHeading())
                .build();

        spikeOne = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(barPose),
                        new Point(spikeOneControlPointOne),
                        new Point(spikeOneControlPointTwo),
                        new Point(spikeOnePose))
                )
                .setLinearHeadingInterpolation(barPose.getHeading(), spikeOnePose.getHeading())
                .build();

        spikeOneToWall = follower.pathBuilder()
                .addPath(new BezierLine(new Point(spikeOnePose), new Point(nearWallPose)))
                .setLinearHeadingInterpolation(spikeOnePose.getHeading(), nearWallPose.getHeading())
                .build();

        spikeTwo = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(nearWallPose),
                        new Point(spikeTwoControlPoint),
                        new Point(spikeTwoPose)
                ))
                .setLinearHeadingInterpolation(nearWallPose.getHeading(), spikeOnePose.getHeading())
                .build();

        spikeTwoToWall = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(spikeTwoPose),
                        new Point(spikeTwoToWallControlPointOne),
                        new Point(spikeTwoToWallControlPointTwo),
                        new Point(wallPose)
                ))
                .setLinearHeadingInterpolation(spikeOnePose.getHeading(), wallPose.getHeading())
                .build();

        wallToBar = follower.pathBuilder()
                .addPath(new BezierLine(new Point(wallPose), new Point(barPose)))
                .setLinearHeadingInterpolation(wallPose.getHeading(), barPose.getHeading())
                .build();

        barToWall = follower.pathBuilder()
                .addPath(new BezierLine(new Point(barPose), new Point(wallPose)))
                .setLinearHeadingInterpolation(barPose.getHeading(), wallPose.getHeading())
                .build();

        wallToBarTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(wallPose), new Point(barPose)))
                .setLinearHeadingInterpolation(wallPose.getHeading(), barPose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(barPose), new Point(wallPose)))
                .setLinearHeadingInterpolation(barPose.getHeading(), wallPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (arm.state() != Arm.State.HOMING) {
                    arm.setTargetPositionPolar(
                            HIGH_BAR_EXTENSION_INCHES,
                            HIGH_BAR_ROTATION_DEGREES
                    );
                    setPathState(1);
                }
                break;
            case 1:
                if (arm.atPosition() && !follower.isBusy()) {
                    follower.followPath(scorePreload , true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    arm.setTargetPositionPolar(
                            HIGH_BAR_EXTENSION_INCHES - 12.0,
                            HIGH_BAR_ROTATION_DEGREES
                    );
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() && arm.atPosition()) {
                    follower.followPath(spikeTwo, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(spikeTwoToWall, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(wallToBar,true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(barToWall, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(wallToBarTwo, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(park, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (follower.isRobotStuck()) follower.breakFollowing();
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        if (!follower.isBusy()) {
            arm.update();
        } else {
            arm.setPower(0.0, 0.0);
        }

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, PathFollowingConstants.class, LocalizerConstants.class);

        arm = new Arm(hardwareMap, OpModeMeta.Flavor.TELEOP);

        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}