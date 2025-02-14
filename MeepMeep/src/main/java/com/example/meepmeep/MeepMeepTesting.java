package com.example.meepmeep;

import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(15.3, 13.9)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(8, -62, Math.PI/2))
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(-4, -33, Math.PI/2), Math.PI/2)
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(0, -30, Math.PI / 2), 0, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
                        return 20;
                    }
                })
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(12, -33, Math.PI/2), 0)

                //splined away from bar
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(35, -30, Math.PI/2), Math.PI/2)
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(42, -16, Math.PI/2), 0)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(48, -24, Math.PI/2), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToSplineHeading(new Pose2d(43, -53, Math.PI/1.8), Math.PI/1.8)
                .setTangent(Math.PI/1.8)
                .splineToLinearHeading(new Pose2d(48, -16, Math.PI/2), 0)


                .setTangent(0)
                .splineToSplineHeading(new Pose2d(56, -24, Math.PI/1.95), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToSplineHeading(new Pose2d(45, -53, Math.PI/1.8), Math.PI/1.5)
                .setTangent(Math.PI/1.5)
                .splineToSplineHeading(new Pose2d(54, -16, Math.PI/2), 0)

                .setTangent(0)
                .splineToSplineHeading(new Pose2d(64, -24, Math.PI/2), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(56, -55, Math.PI/2), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(56, -60, Math.PI / 2), Math.PI / 2, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
                        return 20;
                    }
                })
                .splineToSplineHeading(new Pose2d(-5, -33, -Math.PI/1.999), Math.PI/2)
                .splineToLinearHeading(new Pose2d(-1, -30, -Math.PI/1.999), 0)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(4, -30, -Math.PI/1.999), 0)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(8, -34, -Math.PI / 1.999), -Math.PI / 2, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
                        return 10;
                    }
                })
                .setTangent(-Math.PI/2)
                .splineToSplineHeading(new Pose2d(40, -55, Math.PI/2), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(40, -60, Math.PI / 2), Math.PI / 2, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
                        return 10;
                    }
                })


                .splineToSplineHeading(new Pose2d(-5, -33, -Math.PI/1.999), Math.PI/2)
                .splineToLinearHeading(new Pose2d(-1, -30, -Math.PI/1.999), 0)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(4, -30, -Math.PI/1.999), 0)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(8, -34, -Math.PI / 1.999), -Math.PI / 2, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
                        return 10;
                    }
                })
                .setTangent(-Math.PI/2)
                .splineToSplineHeading(new Pose2d(40, -55, Math.PI/2), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(40, -60, Math.PI / 2), Math.PI / 2, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
                        return 10;
                    }
                })


                .splineToSplineHeading(new Pose2d(-5, -33, -Math.PI/1.999), Math.PI/2)
                .splineToLinearHeading(new Pose2d(-1, -30, -Math.PI/1.999), 0)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(4, -30, -Math.PI/1.999), 0)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(8, -34, -Math.PI / 1.999), -Math.PI / 2, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
                        return 10;
                    }
                })
                .setTangent(-Math.PI/2)
                .splineToSplineHeading(new Pose2d(40, -55, Math.PI/2), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(40, -60, Math.PI / 2), Math.PI / 2, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
                        return 10;
                    }
                })


                .splineToSplineHeading(new Pose2d(-5, -33, -Math.PI/1.999), Math.PI/2)
                .splineToLinearHeading(new Pose2d(-1, -30, -Math.PI/1.999), 0)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(4, -30, -Math.PI/1.999), 0)
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(8, -34, -Math.PI / 1.999), -Math.PI / 2, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
                        return 10;
                    }
                })

                .setTangent(-Math.PI/2)
                .splineTo(new Vector2d(60, -60), 0)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}