package org.firstinspires.ftc.teamcode.hardware.subsystems.arm;

import androidx.annotation.NonNull;

public final class CartesianPosition {
    public final double horizontalInches;
    public final double verticalInches;

    public CartesianPosition(double horizontalInches, double verticalInches) {
        this.horizontalInches = horizontalInches;
        this.verticalInches = verticalInches;
    }
}
