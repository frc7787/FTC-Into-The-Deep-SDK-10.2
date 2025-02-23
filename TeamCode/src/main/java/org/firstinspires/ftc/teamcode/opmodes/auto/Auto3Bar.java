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
import org.firstinspires.ftc.teamcode.roadrunner.actions.MoveToPositionAction;
import org.jetbrains.annotations.NotNull;


@Autonomous
public class Auto3Bar extends LinearOpMode {
    private final Pose2d initialPose = new Pose2d(8, -62, -Math.PI / 2);

    @Override public void runOpMode() {

        ElapsedTime elapsedTime = new ElapsedTime();

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder startToBarAction = drive.actionBuilder(initialPose)
                .setTangent(Math.PI/2)
                .splineTo(new Vector2d(-5, -26), Math.PI/2);

        TrajectoryActionBuilder trajectory2 = startToBarAction.endTrajectory().fresh()
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
                .splineToSplineHeading(new Pose2d(62.75, -48, Math.PI/2), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(50, -53, Math.PI/2), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(50, -64, Math.PI / 2), Math.PI / 2, (pose2dDual, posePath, v) -> 12.5)
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(2, -23.5, -Math.PI / 1.999), Math.PI/2);

//              break in paths



        TrajectoryActionBuilder trajectory3 = trajectory2.endTrajectory().fresh()
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(12, -38, -Math.PI/1.999), 0)
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(40, -50, Math.PI/2), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(40, -64, Math.PI / 2), Math.PI / 2, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
                        return 12.5;
                    }
                })

                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(0, -23, -Math.PI / 1.999), Math.PI/2);

        TrajectoryActionBuilder trajectory4 = trajectory3.endTrajectory().fresh()
                .setTangent(-Math.PI/2)
                .splineTo(new Vector2d(60, -58), 0, (pose2dDual, posePath, v) -> 100);

        Arm arm = new Arm(hardwareMap);
        arm.setAutoHoming();
        arm.setIntakePosition(0.08);

        boolean init = false;

        while (opModeInInit()) {
            if (!(arm.state() == Arm.State.HOMING)) {
                if (!init) {
                    init = true;
                    arm.setTargetPositionPolar(2.5, -47.5);
                }
            }

            arm.update();

            arm.globalDebug(telemetry);
            arm.positionDebug(telemetry);
            telemetry.update();

            if (arm.atPosition()) {
                arm.stop();
            }
        }

        waitForStart();

        Actions.runBlocking(
              new SequentialAction(
                      new MoveToPositionAction(arm, 24.5, -6.4, 1.5, telemetry),
                      new ParallelAction(
                              startToBarAction.build()

                      ),
                      new MoveToPositionAction(arm, 10.0, -6.4, 0.7, telemetry),
                      new ParallelAction(
                              trajectory2.build(),
                              new SequentialAction(
                                      new SleepAction(8),
                                      new MoveToPositionAction(arm, -4, -15.2, 1, telemetry),
                                      new SleepAction(1.75),
                                      new MoveToPositionAction(arm, 24.5, -6.4, 1.5, telemetry)
                              )

                      ),
                      new MoveToPositionAction(arm, 10.0, -6.5, 0.7, telemetry),
                      new ParallelAction(
                              trajectory3.build(),
                              new SequentialAction(
                                      new SleepAction(1.75),
                                      new MoveToPositionAction(arm, -4, -15.2, 2, telemetry),
                                      new SleepAction(0.55),
                                      new MoveToPositionAction(arm, 24.5, -6.4, 1.5, telemetry)
                              )
                      ),
                      new MoveToPositionAction(arm, 10, -6.5, 0.7, telemetry),
                      trajectory4.build()

              )

        );




    }


}