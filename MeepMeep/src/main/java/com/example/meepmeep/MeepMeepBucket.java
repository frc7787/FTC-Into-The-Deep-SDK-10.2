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

public class MeepMeepBucket {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(15.3, 13.9)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-8, -62, -Math.PI/2))
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(4, -30, -Math.PI/2), Math.PI/2)
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(0, -28, -Math.PI / 2), Math.PI, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
                        return 20;
                    }
                })
                .setTangent(Math.PI)
                .splineToSplineHeading(new Pose2d(-8, -33, -Math.PI/2), -Math.PI/2)
                .splineTo(new Vector2d(-48, -36), Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToSplineHeading(new Pose2d(-48, -48, -3*Math.PI/4), -3*Math.PI/4)
                .setTangent(Math.PI/2)
                .splineToSplineHeading(new Pose2d(-58, -36, Math.PI/2), Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToSplineHeading(new Pose2d(-48, -48, -3*Math.PI/4), -Math.PI/4)
                .setTangent(Math.PI/2)
                .splineToSplineHeading(new Pose2d(-60, -36, 11*Math.PI/16), 5*Math.PI/8)


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}