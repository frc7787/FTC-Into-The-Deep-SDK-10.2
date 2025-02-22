package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.actions.HomeArmAction;
import org.firstinspires.ftc.teamcode.roadrunner.actions.RetractArmAction;
import org.jetbrains.annotations.NotNull;


@Autonomous
public class Auto3Bar extends LinearOpMode {
    private final Pose2d initialPose = new Pose2d(8, -62, -Math.PI / 2);

    @Override public void runOpMode() {

        ElapsedTime elapsedTime = new ElapsedTime();

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder startToBarAction = drive.actionBuilder(initialPose)
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(-4, -30, -Math.PI/2), Math.PI/2)
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(0, -28, -Math.PI / 2), 0, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
                        return 20;
                    }
                })
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(8, -33, -Math.PI/2), -Math.PI/2)

                //splined away from bar
                .setTangent(-Math.PI/2)
                .splineTo(new Vector2d(35, -30), Math.PI/2)
                .setTangent(Math.PI/2)
                .splineToSplineHeading(new Pose2d(42, -14, Math.PI/2), 0)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(48, -24, Math.PI/2), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToSplineHeading(new Pose2d(43, -53, Math.PI/2), Math.PI)
                .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(48, -14, Math.PI/2), 0)


                .setTangent(0)
                .splineToSplineHeading(new Pose2d(56, -24, Math.PI/2), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToSplineHeading(new Pose2d(49, -53, Math.PI/2), Math.PI)
                .setTangent(Math.PI)
                .splineToSplineHeading(new Pose2d(54, -14, Math.PI/2), 0)

                .setTangent(0)
                .splineToSplineHeading(new Pose2d(62.75, -24, Math.PI/2), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(56, -55, Math.PI/2), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(56, -63, Math.PI / 2), Math.PI / 2, (pose2dDual, posePath, v) -> 20)
                .splineToSplineHeading(new Pose2d(-4, -26, -Math.PI/1.999), Math.PI/2)
//                .splineToLinearHeading(new Pose2d(-1, -30, -Math.PI/1.999), 0)
//                .setTangent(0)
//                .splineToLinearHeading(new Pose2d(-4, -26, -Math.PI/1.999), 0)
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(0, -24.5, -Math.PI / 1.999), 0, (pose2dDual, posePath, v) -> 10)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(12, -31, -Math.PI/1.999), 0)
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(40, -55, Math.PI/2), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(40, -63, Math.PI / 2), Math.PI / 2, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
                        return 20;
                    }
                })


                .splineToSplineHeading(new Pose2d(-4, -26.5, -Math.PI/1.999), Math.PI/2)
//                .splineToLinearHeading(new Pose2d(-1, -30, -Math.PI/1.999), 0)
//                .setTangent(0)
//                .splineToLinearHeading(new Pose2d(-4, -26, -Math.PI/1.999), 0)
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(0, -24, -Math.PI / 1.999), 0, (pose2dDual, posePath, v) -> 10)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(12, -33, -Math.PI/1.999), 0)
                .setTangent(0)


                .setTangent(0)
                .splineToLinearHeading(new Pose2d(60, -53, -Math.PI/1.999), -Math.PI/2, (pose2dDual, posePath, v) -> 80);
        Arm arm = new Arm(hardwareMap);
        arm.setAutoHoming();
        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new HomeArmAction(arm),
                        new ParallelAction(
                                startToBarAction.build(),
                                new SequentialAction(
                                        new RetractArmAction(arm, 1.5, 1.0),
                                        new SleepAction(0.3),
                                        new RetractArmAction(arm, 0.8, -1)


                                )
                        )
                )
        );




    }


}