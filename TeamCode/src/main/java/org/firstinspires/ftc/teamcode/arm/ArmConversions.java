package org.firstinspires.ftc.teamcode.arm;

import static org.firstinspires.ftc.teamcode.arm.ArmConstants.*;

import androidx.annotation.NonNull;

public final class ArmConversions {

    public static double rotationTicksToDegrees(double ticks) {
       return (ticks / ROTATION_TICKS_PER_DEGREE) + ROTATION_STARTING_ANGLE;
    }

    public static double extensionTicksToInches(double ticks) {
        return (ticks / EXTENSION_TICKS_PER_INCH);
    }

    /**
     * Converts cartesian coordinates of the arm to polar coordinates
     * @param xInches The x position of the arm
     * @param yInches The y position of the arm
     * @return The polar coordinates of the arm (Extension, Angle)
     */
    @NonNull static double[] cartesianToPolar(double xInches, double yInches) {
        double extensionInches = Math.sqrt(
                Math.pow(xInches, 2.0) +
                        Math.pow(yInches, 2.0) +
                        Math.pow(1.5, 2)
        ) - MIN_EXT_INCHES;

        double rotationDegrees = Math.toDegrees(
                Math.atan(yInches / xInches) -
                        Math.atan(1.5 / (extensionInches + MIN_EXT_INCHES))
        );

        return new double[] {extensionInches, rotationDegrees};
    }

    /**
     * Converts the polar coordinates of the arm to cartesian coordinates
     * @param extensionInches The extension of the arm in inches
     * @param rotationDegrees The rotation of the arm in degrees
     * @return The cartesian coordinates of the arm (X,Y)
     */
    @NonNull static double[] polarToCartesian(double extensionInches, double rotationDegrees) {
        extensionInches += MIN_EXT_INCHES;
        double thetaRadians = Math.toRadians(rotationDegrees) + Math.atan(1.5 / extensionInches);

        return new double[]{
                extensionInches * Math.cos(thetaRadians),
                extensionInches * Math.sin(thetaRadians)
        };
    }

    @NonNull static double[] robotToArmCentric(@NonNull double[] armCentricCartesianCoordinates) {
        return new double[]{
                armCentricCartesianCoordinates[0] + ROTATION_HORIZONTAL_OFFSET_INCHES,
                armCentricCartesianCoordinates[1] + ROTATION_VERTICAL_OFFSET_INCHES
        };
    }

    @NonNull static double[] armToRobotCentric(@NonNull double[] robotCentricCartesianCoordinates) {
        return new double[]{
                robotCentricCartesianCoordinates[0] - ROTATION_HORIZONTAL_OFFSET_INCHES,
                robotCentricCartesianCoordinates[1] - ROTATION_VERTICAL_OFFSET_INCHES
        };
    }
}
