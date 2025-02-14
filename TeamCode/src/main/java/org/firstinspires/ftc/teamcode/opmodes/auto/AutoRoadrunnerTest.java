package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous
public class AutoRoadrunnerTest extends LinearOpMode {
    private final Pose2d initialPose = new Pose2d(8, -62, Math.PI / 2);

    @Override public void runOpMode() {

        ElapsedTime elapsedTime = new ElapsedTime();

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder startToBarAction = drive.actionBuilder(initialPose)
                .setTangent(Math.PI / 2)
                .splineTo(new Vector2d(4, -27), Math.PI/2);

        TrajectoryActionBuilder pushFirstTwoSamplesIntoToObservationZoneAction = startToBarAction.endTrajectory().fresh()
                // To First Sample
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(26, -36, Math.PI), Math.PI / 2)
                .setTangent(Math.PI / 2)
                .splineToSplineHeading(new Pose2d(30, -21, -Math.PI /2), Math.PI/2)
                .setTangent(Math.PI/2)
                .splineToConstantHeading(new Vector2d(47, -10), 0)
                .setTangent(-Math.PI/2)
                .splineToConstantHeading(new Vector2d(47, -56), -Math.PI / 2)
                // Push Sample Into Zone
                .setTangent(Math.PI / 2)
                .splineToSplineHeading(new Pose2d(47, -20, -Math.PI /2), Math.PI / 4)
                .setTangent(Math.PI/4)
                .splineToLinearHeading(new Pose2d(50, -16, -Math.PI /4), Math.PI / 2)
                // Push Second Sample Into Zone
                .setTangent(-Math.PI/4)
                .splineTo(new Vector2d(56, -21), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToSplineHeading(new Pose2d(58, -56, -Math.PI /2), -Math.PI / 2);

        TrajectoryActionBuilder attachToFirstSample = pushFirstTwoSamplesIntoToObservationZoneAction.endTrajectory().fresh()
                .waitSeconds(0.3)
                .setTangent(-Math.PI/2)
                .lineToY(-68, null, new ProfileAccelConstraint(-70.0, 70.0));

        TrajectoryActionBuilder firstSamplePickupToBar = attachToFirstSample.endTrajectory().fresh()
                .waitSeconds(0.3)
                .setTangent(-Math.PI/2)
                .lineToY(-67, null, new ProfileAccelConstraint(-70.0, 70.0))
                .setTangent(Math.PI/2)
                .splineToSplineHeading(new Pose2d(-2, -29, Math.PI /1.999), Math.PI / 2);

        TrajectoryActionBuilder barToSecondSamplePickup = firstSamplePickupToBar.endTrajectory().fresh()
                .setTangent(-Math.PI/2)
                .splineToSplineHeading(new Pose2d(58, -56, -Math.PI / 2), -Math.PI / 2);

        TrajectoryActionBuilder attachToSecondSample = barToSecondSamplePickup.endTrajectory().fresh()
                .waitSeconds(0.3)
                .setTangent(-Math.PI/2)
                .lineToY(-71, null, new ProfileAccelConstraint(-70.0, 70.0));

        TrajectoryActionBuilder secondSamplePickupToBar = attachToSecondSample.endTrajectory().fresh()
                .setTangent(Math.PI / 2)
                .splineToSplineHeading(new Pose2d(-0.5, -31, Math.PI / 2), Math.PI / 2);

        TrajectoryActionBuilder bookItToObservationZoneBuilder = secondSamplePickupToBar.endTrajectory().fresh()
                .setTangent(-Math.PI/2)
                .splineTo(new Vector2d(30, -60), 0)
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(60, -66), 0);


        waitForStart();




        Actions.runBlocking(startToBarAction.build());


        Actions.runBlocking(pushFirstTwoSamplesIntoToObservationZoneAction.build());



        Actions.runBlocking(attachToFirstSample.build());


        Actions.runBlocking(firstSamplePickupToBar.build());


        Actions.runBlocking(barToSecondSamplePickup.build());



        Actions.runBlocking(attachToSecondSample.build());


        Actions.runBlocking(secondSamplePickupToBar.build());

        Actions.runBlocking(bookItToObservationZoneBuilder.build());
    }


}