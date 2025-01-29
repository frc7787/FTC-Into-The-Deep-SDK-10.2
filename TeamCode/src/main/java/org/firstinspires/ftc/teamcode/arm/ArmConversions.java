package org.firstinspires.ftc.teamcode.arm;

final class ArmConversions {

    public static double opticalTrackerReadingToInches(double reading) {
        return reading / ArmConstants.SPARK_FUN_OTOS_UNITS_PER_INCH;
    }
}
