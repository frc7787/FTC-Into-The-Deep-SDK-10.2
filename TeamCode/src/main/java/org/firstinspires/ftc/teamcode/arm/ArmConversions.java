package org.firstinspires.ftc.teamcode.arm;

import static org.firstinspires.ftc.teamcode.arm.ArmConstants.*;

import androidx.annotation.NonNull;

public final class ArmConversions {

    public static double potentiometerVoltageToDegrees(double voltage) {
        return voltage / POTENTIOMETER_VOLTS_PER_DEGREE;
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
}
