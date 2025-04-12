package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
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

@Autonomous(group = "Examples")
public class Bar extends OpMode {
    public static volatile double HIGH_BAR_EXTENSION_INCHES = 35.0;
    public static volatile double HIGH_BAR_ROTATION_DEGREES = 87.0;

    public static volatile double AUTO_INIT_EXTENSION_INCHES = 11.0;
    public static volatile double AUTO_INIT_ROTATION_DEGREES = 45.0;

    private Arm arm;

    private double extensionInches;

    private boolean updateArm;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private Pose startPose = new Pose(8.0, 55.0, Math.toRadians(90.0));
    private Pose barPose = new Pose(41.0, 68.0, Math.toRadians(180.0));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path park;

    private PathChain line1,
                      line2,
                      line3,
                      line4,
                      line5,
                      line6,
                      line7,
                      line8,
                      line9,
                      line10,
                      line11,
                      line12,
                      line13,
                      line14,
                      line15,
                      line16;

    public void buildPaths() {
        PathBuilder builder = follower.pathBuilder();

        line1 = builder
                .addPath(new BezierLine(new Point(startPose), new Point(barPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), barPose.getHeading())
                .build();

        line2 = builder
                .addPath(new BezierLine(new Point(41.064, 68.809), new Point(29.410, 37.734)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-45))
                .build();

        line3 = builder
                .addPath(new BezierLine(new Point(29.410, 37.734), new Point(28.023, 33.572)))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-135))
                .build();

        line4 = builder
                .addPath(new BezierLine(new Point(28.023, 33.572), new Point(30.243, 30.243)))
                .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(-45))
                .build();

        line5 = builder
                .addPath(new BezierLine(new Point(30.243, 30.243), new Point(28.023, 27.191)))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-135))
                .build();

        line6 = builder
                .addPath(new BezierLine(new Point(28.023, 27.191), new Point(38.289, 19.699)))
                .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(-60))
                .build();

        line7 = builder
                .addPath(new BezierLine(new Point(38.289, 19.699), new Point(25.803, 21.087)))
                .setLinearHeadingInterpolation(Math.toRadians(-60), Math.toRadians(-135))
                .build();

        line8 = builder
                .addPath(new BezierLine(new Point(25.803, 21.087), new Point(8.046, 31.630)))
                .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(0))
                .build();

        line9 = builder
                .addPath(new BezierLine(new Point(8.046, 31.630), new Point(41.618, 76.855)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        line10 = builder
                .addPath(new BezierLine(new Point(41.618, 76.855), new Point(8.046, 31.630)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();

        line11 = builder
                .addPath(new BezierLine(new Point(8.046, 31.630), new Point(41.341, 74.081)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        line12 = builder
                .addPath(new BezierLine(new Point(41.341, 74.081), new Point(8.046, 31.630)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();

        line13 = builder
                .addPath(new BezierLine(new Point(8.046, 31.630), new Point(41.341, 71.306)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        line14 = builder
                .addPath(new BezierLine(new Point(41.341, 71.306), new Point(8.046, 31.630)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();

        line15 = builder
                .addPath(new BezierLine(new Point(8.046, 31.630), new Point(41.064, 66.590)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        line16 = builder
                .addPath(new BezierLine(new Point(41.064, 66.590), new Point(12.208, 24.139)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 1:
                follower.followPath(line1, true);
                if (!follower.isBusy()) setPathState(2);
                break;
            case 2:
                follower.followPath(line2, true);
                if (!follower.isBusy()) setPathState(3);
                break;
            case 3:
                follower.followPath(line3, true);
                if (!follower.isBusy()) setPathState(4);
                break;
            case 4:
                follower.followPath(line4, true);
                if (!follower.isBusy()) setPathState(5);
                break;
            case 5:
                follower.followPath(line5, true);
                if (!follower.isBusy()) setPathState(6);
                break;
            case 6:
                follower.followPath(line6, true);
                if (!follower.isBusy()) setPathState(7);
                break;
            case 7:
                follower.followPath(line7, true);
                if (!follower.isBusy()) setPathState(8);
                break;
            case 8:
                follower.followPath(line8, true);
                if (!follower.isBusy()) setPathState(9);
                break;
            case 9:
                follower.followPath(line9, true);
                if (!follower.isBusy()) setPathState(10);
                break;
            case 10:
                follower.followPath(line10, true);
                if (!follower.isBusy()) setPathState(11);
                break;
            case 11:
                follower.followPath(line11, true);
                if (!follower.isBusy()) setPathState(12);
                break;
            case 12:
                follower.followPath(line12, true);
                if (!follower.isBusy()) setPathState(13);
                break;
            case 13:
                follower.followPath(line13, true);
                if (!follower.isBusy()) setPathState(14);
                break;
            case 14:
                follower.followPath(line14, true);
                if (!follower.isBusy()) setPathState(15);
                break;
            case 15:
                follower.followPath(line15, true);
                if (!follower.isBusy()) setPathState(16);
                break;
            case 16:
                follower.followPath(line16, true);
                if (!follower.isBusy()) setPathState(17);
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override public void loop() {
        telemetry.addData("Start Position", extensionInches);

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();
        arm.globalDebug(telemetry);
        arm.positionDebug(telemetry);

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

        updateArm = true;

        follower = new Follower(hardwareMap, PathFollowingConstants.class, LocalizerConstants.class);
        follower.setStartingPose(startPose);

        buildPaths();

        double startPosition = 0.0;
        actionTimer = new Timer();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override public void start() {
        opmodeTimer.resetTimer();
        actionTimer.resetTimer();
        setPathState(1);
        follower.setPose(startPose);
    }
}