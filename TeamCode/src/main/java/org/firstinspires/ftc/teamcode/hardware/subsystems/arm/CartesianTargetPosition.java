package org.firstinspires.ftc.teamcode.hardware.subsystems.arm;

import androidx.annotation.NonNull;

/**
 * Class to represent a target position of the arm in cartesian coordinates
 */
public final class CartesianTargetPosition {
    public final double horizontalInches;
    public final double verticalInches;

    @NonNull public final MovementOrdering movementOrdering;
    @NonNull public final ReferenceFrame referenceFrame;

    /**
     * Creates a new cartesian target position.
     * To change the movement ordering or reference frame without having to construct a new object
     * see {@link CartesianTargetPosition#withMovementOrdering(MovementOrdering)} and
     * {@link CartesianTargetPosition#withReferenceFrame(ReferenceFrame)}
     * @param horizontalInches
     * @param verticalInches
     * @param movementOrdering
     * @param referenceFrame
     */
    public CartesianTargetPosition(
            double horizontalInches,
            double verticalInches,
            @NonNull MovementOrdering movementOrdering,
            @NonNull ReferenceFrame referenceFrame
    ) {
        this.horizontalInches = horizontalInches;
        this.verticalInches = verticalInches;
        this.movementOrdering = movementOrdering;
        this.referenceFrame = referenceFrame;
    }

    /**
     * @return The polar representation of the cartesian target position. Converted using
     *         {@link Arm#cartesianToPolar(double, double)}.
     */
    @NonNull public PolarTargetPosition asPolarTargetPosition() {
        double[] polarTargetCoordinates = Arm.cartesianToPolar(horizontalInches, verticalInches);

        return new PolarTargetPosition(
                polarTargetCoordinates[0],
                polarTargetCoordinates[1],
                movementOrdering
        );
    }


    @NonNull public CartesianTargetPosition withMovementOrdering(
            @NonNull MovementOrdering movementOrdering
    ) {
        return new CartesianTargetPosition(
                horizontalInches,
                verticalInches,
                movementOrdering,
                referenceFrame
        );
    }

    @NonNull public CartesianTargetPosition withReferenceFrame(
            @NonNull ReferenceFrame referenceFrame
    ) {
        return new CartesianTargetPosition(
                horizontalInches,
                verticalInches,
                movementOrdering,
                referenceFrame
        );
    }
}
