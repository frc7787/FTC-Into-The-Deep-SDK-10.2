package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.jetbrains.annotations.NotNull;


@Autonomous
public class AutoRoadrunnerTest extends LinearOpMode {
    private final Pose2d initialPose = new Pose2d(8, -62, -Math.PI / 2);

    @Override public void runOpMode() {

        ElapsedTime elapsedTime = new ElapsedTime();

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder startToBarAction = drive.actionBuilder(initialPose)
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(-4, -33, -Math.PI/2), Math.PI/2)
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(0, -28, -Math.PI / 2), 0, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
                        return 20;
                    }
                })
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(12, -33, -Math.PI/2), 0)

                //splined away from bar
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(35, -30, Math.PI/2), Math.PI/2)
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(42, -14, Math.PI/2), 0)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(48, -24, Math.PI/2), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToSplineHeading(new Pose2d(43, -53, Math.PI/1.8), Math.PI/1.8)
                .setTangent(Math.PI/1.8)
                .splineToLinearHeading(new Pose2d(48, -14, Math.PI/2), 0)


                .setTangent(0)
                .splineToSplineHeading(new Pose2d(56, -24, Math.PI/2.05), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToSplineHeading(new Pose2d(45, -53, Math.PI/2.2), Math.PI/1.5)
                .setTangent(Math.PI/1.5)
                .splineToSplineHeading(new Pose2d(54, -14, Math.PI/2), 0)

                .setTangent(0)
                .splineToSplineHeading(new Pose2d(64, -24, Math.PI/2), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(56, -55, Math.PI/2), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(56, -62, Math.PI / 2), Math.PI / 2, (pose2dDual, posePath, v) -> 20)
                .splineToSplineHeading(new Pose2d(-4, -33, -Math.PI/1.999), Math.PI/2)
//                .splineToLinearHeading(new Pose2d(-1, -30, -Math.PI/1.999), 0)
//                .setTangent(0)
//                .splineToLinearHeading(new Pose2d(-4, -26, -Math.PI/1.999), 0)
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(0, -28, -Math.PI / 1.999), 0, (pose2dDual, posePath, v) -> 10)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(12, -33, -Math.PI/1.999), 0)
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(40, -55, Math.PI/2), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(40, -62, Math.PI / 2), Math.PI / 2, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
                        return 10;
                    }
                })


                .splineToSplineHeading(new Pose2d(-4, -33, -Math.PI/1.999), Math.PI/2)
//                .splineToLinearHeading(new Pose2d(-1, -30, -Math.PI/1.999), 0)
//                .setTangent(0)
//                .splineToLinearHeading(new Pose2d(-4, -26, -Math.PI/1.999), 0)
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(0, -28, -Math.PI / 1.999), 0, (pose2dDual, posePath, v) -> 10)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(12, -33, -Math.PI/1.999), 0)
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(40, -55, Math.PI/2), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(40, -62, Math.PI / 2), Math.PI / 2, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
                        return 10;
                    }
                })


                .splineToSplineHeading(new Pose2d(-4, -33, -Math.PI/1.999), Math.PI/2)
//                .splineToLinearHeading(new Pose2d(-1, -30, -Math.PI/1.999), 0)
//                .setTangent(0)
//                .splineToLinearHeading(new Pose2d(-4, -26, -Math.PI/1.999), 0)
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(0, -28, -Math.PI / 1.999), 0, (pose2dDual, posePath, v) -> 10)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(12, -33, -Math.PI/1.999), 0)
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(40, -55, Math.PI/2), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(40, -62, Math.PI / 2), Math.PI / 2, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
                        return 10;
                    }
                })


                .splineToSplineHeading(new Pose2d(-4, -33, -Math.PI/1.999), Math.PI/2)
//                .splineToLinearHeading(new Pose2d(-1, -30, -Math.PI/1.999), 0)
//                .setTangent(0)
//                .splineToLinearHeading(new Pose2d(-4, -26, -Math.PI/1.999), 0)
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(0, -28, -Math.PI / 1.999), 0, (pose2dDual, posePath, v) -> 10)
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(5, -33, -Math.PI/1.999), -Math.PI/2)


                .setTangent(-Math.PI/2)
                .splineTo(new Vector2d(60, -60), 0);

        waitForStart();

        Actions.runBlocking(startToBarAction.build());

    }


}