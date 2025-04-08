package org.firstinspires.ftc.teamcode.hardware.subsystems.arm;

/** Enum to represent in what order the arm should move. */
public enum MovementOrdering {
    /** Moves the rotation to position then the extension. */
    ROTATION_THEN_EXTENSION,
    /** Moves the extension to position then the rotation. */
    EXTENSION_THEN_ROTATION,
    /** Moves the extension and rotation at the same time. */
    EXTENSION_AND_ROTATION
}