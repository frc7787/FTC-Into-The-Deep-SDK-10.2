package org.firstinspires.ftc.teamcode.hardware.subsystems.arm;

import androidx.annotation.NonNull;

public final class PolarTargetPosition {
    public final double extensionInches;
    public final double rotationDegrees;

    @NonNull public final MovementOrdering movementOrdering;

    public PolarTargetPosition(
            double extensionInches,
            double rotationDegrees,
            @NonNull MovementOrdering movementOrdering
    ) {
        this.extensionInches = extensionInches;
        this.rotationDegrees = rotationDegrees;
        this.movementOrdering = movementOrdering;
    }

    @NonNull public CartesianTargetPosition asCartesianTargetPosition() {
        double[] polarTargetCoordinates = Arm.polarToCartesian(extensionInches, rotationDegrees);

        return new CartesianTargetPosition(
                polarTargetCoordinates[0],
                polarTargetCoordinates[1],
                movementOrdering,
                ReferenceFrame.ARM_CENTRIC
        );
    }

    @NonNull public PolarTargetPosition withMovementOrdering(
            @NonNull MovementOrdering movementOrdering
    ) {
        return new PolarTargetPosition(
                extensionInches,
                rotationDegrees,
                movementOrdering
        );
    }
}
