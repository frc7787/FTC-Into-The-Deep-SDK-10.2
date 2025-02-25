package org.firstinspires.ftc.teamcode.opmodes.auto;

import androidx.annotation.NonNull;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.actions.HomeArmAction;
import org.firstinspires.ftc.teamcode.roadrunner.actions.MoveToPositionAction;
import org.firstinspires.ftc.teamcode.roadrunner.actions.RetractArmAction;
import org.jetbrains.annotations.NotNull;


@Autonomous

public class Auto4Bar extends LinearOpMode {
    private final Pose2d initialPose = new Pose2d(8, -62, -Math.PI / 2);

    @Override public void runOpMode() {

        ElapsedTime elapsedTime = new ElapsedTime();

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Arm arm = new Arm(hardwareMap);
        arm.setAutoHoming();
        arm.setIntakePosition(0.08);

        boolean init = false;

        while (opModeInInit()) {
            if (isStopRequested()) return;

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

        TrajectoryActionBuilder startToBarAction = drive.actionBuilder(initialPose)
                .afterTime(0.01, new MoveToPositionAction(arm, 24.0, -6.4, 1.5, telemetry))
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(2, -28, -Math.PI/2), Math.PI/2)
                .setTangent(Math.PI/2)
                .afterTime(0.01, new MoveToPositionAction(arm, 10, -6.5, 1.0, telemetry))
                .splineToLinearHeading(new Pose2d(6, -28, -Math.PI / 2), 0, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
                        return 20;
                    }
                })
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(10, -33, -Math.PI/2), -Math.PI/2)
                .afterTime(0.1, new MoveToPositionAction(arm, -2, -15.5, 0.75, telemetry))
                //splined away from bar
                .setTangent(-Math.PI/2)
                .splineTo(new Vector2d(35, -30), Math.PI/2)
                .setTangent(Math.PI/2)
                .splineToSplineHeading(new Pose2d(42, -14, Math.PI/2), 0)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(47, -24, Math.PI/2), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(47, -47, Math.PI/2), Math.PI/2)
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(49, -14, Math.PI / 2), Math.PI / 2, (pose2dDual, posePath, v) -> 38)
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(55, -10, Math.PI/2), 0)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(55, -38, Math.PI/2), -Math.PI/2, (pose2dDual, posePath, v) -> 38)
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(43, -55, Math.PI/2), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .afterTime(0.9, new MoveToPositionAction(arm, 24.0, -6.4, 1.5, telemetry))
                .splineToLinearHeading(new Pose2d(43, -63.5, Math.PI / 2), Math.PI / 2, (pose2dDual, posePath, v) -> 10)

                .splineToSplineHeading(new Pose2d(-2, -26, -Math.PI/1.999), Math.PI/2)
//                .splineToLinearHeading(new Pose2d(-1, -30, -Math.PI/1.999), 0)
//                .setTangent(0)
//                .splineToLinearHeading(new Pose2d(-4, -26, -Math.PI/1.999), 0)
                .setTangent(Math.PI/2)
                .afterTime(0.3, new MoveToPositionAction(arm, 10, -6.4, 1.0, telemetry))
                .splineToLinearHeading(new Pose2d(2, -24.5, -Math.PI / 1.999), 0, (pose2dDual, posePath, v) -> 10)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(12, -31, -Math.PI/1.999), 0)
                .afterTime(0.1, new MoveToPositionAction(arm, -2, -15.5, 0.75, telemetry))
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(40, -55, Math.PI/2), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .afterTime(0.8, new MoveToPositionAction(arm, 24.0, -6.4, 1.5, telemetry))
                .splineToLinearHeading(new Pose2d(40, -63, Math.PI / 2), Math.PI / 2, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
                        return 10;
                    }
                })

                .afterTime(0.1, new MoveToPositionAction(arm, 23.65, -6.5, 1.5, telemetry))
                .splineToSplineHeading(new Pose2d(-6, -26, -Math.PI/1.999), Math.PI/2)
//                .splineToLinearHeading(new Pose2d(-1, -30, -Math.PI/1.999), 0)
//                .setTangent(0)
//                .splineToLinearHeading(new Pose2d(-4, -26, -Math.PI/1.999), 0)
                .setTangent(Math.PI/2)
                .afterTime(0.7, new MoveToPositionAction(arm, 10, -6.5, 1.0, telemetry))
                .splineToLinearHeading(new Pose2d(-2, -24.5, -Math.PI / 1.999), 0, (pose2dDual, posePath, v) -> 10)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(12, -31, -Math.PI/1.999), 0)
                .afterTime(0.1, new MoveToPositionAction(arm, -2, -15.5, 0.75, telemetry))
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(40, -55, Math.PI/2), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .afterTime(1, new MoveToPositionAction(arm, 24.0, -6.5, 1.5, telemetry))
                .splineToLinearHeading(new Pose2d(40, -63, Math.PI / 2), Math.PI / 2, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
                        return 10;
                    }
                })

                .splineToSplineHeading(new Pose2d(-2, -25, -Math.PI/1.999), Math.PI/2)

                .afterTime(0.2, new MoveToPositionAction(arm, 10, -6.5, 1.0, telemetry))
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(2, -22.5, -Math.PI / 1.999), 0, (pose2dDual, posePath, v) -> 10)

                .setTangent(0)
                .splineToSplineHeading(new Pose2d(9, -33, -Math.PI/1.999), -Math.PI/2)

                .afterTime(0.1, new MoveToPositionAction(arm, -2, -15.2, 0.75, telemetry))
                .setTangent(-Math.PI/2)
                .splineTo(new Vector2d(60, -60), 0, (pose2dDual, posePath, v) -> 80);


        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        startToBarAction.build()
                )
        );
    }
}