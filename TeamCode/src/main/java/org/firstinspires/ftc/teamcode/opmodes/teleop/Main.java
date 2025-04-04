package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedropathing.constants.*;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;

import dev.frozenmilk.dairy.core.util.OpModeLazyCell;

@TeleOp(group = "$")
@Config
public final class Main extends OpMode {

    // ---------------------------------------------------------------------------------------------
    // Positions

    public static volatile double HIGH_BAR_EXTENSION_INCHES = 0.0;
    public static volatile double HIGH_BAR_ROTATION_DEGREES = 0.0;

    public static volatile double LOW_BAR_EXTENSION_INCHES = 0.0;
    public static volatile double LOW_BAR_ROTATION_DEGREES = 0.0;

    public static volatile double HIGH_BUCKET_EXTENSION_INCHES = 0.0;
    public static volatile double HIGH_BUCKET_ROTATION_DEGREES = 0.0;

    public static volatile double LOW_BUCKET_EXTENSION_INCHES = 0.0;
    public static volatile double LOW_BUCKET_ROTATION_DEGREES = 0.0;

    public static volatile double SUB_EXTENSION_INCHES = 0.0;
    public static volatile double SUB_ROTATION_DEGREES = 0.0;

    public static volatile double HOME_EXTENSION_INCHES = 0.0;
    public static volatile double HOME_ROTATION_DEGREES = 0.0;

    public static volatile double WALL_PRIME_EXTENSION_INCHES = 0.0;
    public static volatile double WALL_PRIME_ROTATION_DEGREES = 0.0;

    public static volatile double WALL_PICKUP_EXTENSION_INCHES = 0.0;

    public static volatile double HANG_EXTENSION_INCHES = 0.0;
    public static volatile double HANG_ROTATION_DEGREES = 0.0;

    private Arm arm;
    private Intake intake;
    private Hanger hanger;

    private Follower mecanumDrive;

    private TeleOpState teleOpState;

    private Gamepad currentGamepad1, previousGamepad1;

    @Override public void init() {
        arm = new Arm(hardwareMap, OpModeMeta.Flavor.TELEOP);
        intake = new Intake(hardwareMap);
        hanger = new Hanger(hardwareMap);
        mecanumDrive = new Follower(hardwareMap, PathFollowingConstants.class, LocalizerConstants.class);
        mecanumDrive.setStartingPose(new Pose(0.0, 0.0, 0.0));
        teleOpState = TeleOpState.NORMAL;
        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
    }

    @Override public void start() { mecanumDrive.startTeleopDrive(); }

    @Override public void loop() {
        drive();

        // Todo add hang button

        switch (teleOpState) {
            case NORMAL:
                if (gamepad2.right_bumper) {
                    intake.open();
                } else if (gamepad2.left_bumper) {
                    intake.close();
                }

                if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                    arm.setTargetPositionPolar(SUB_EXTENSION_INCHES, SUB_ROTATION_DEGREES);
                    teleOpState = TeleOpState.NORMAL;
                    break;
                }

                if (gamepad2.circle) {
                    arm.setTargetPositionPolar(HIGH_BAR_EXTENSION_INCHES, HIGH_BAR_ROTATION_DEGREES);
                } else if (gamepad2.triangle) {
                    arm.setTargetPositionPolar(HIGH_BUCKET_EXTENSION_INCHES, HIGH_BUCKET_ROTATION_DEGREES);
                } else if (gamepad2.square) {
                    arm.setTargetPositionPolar(WALL_PRIME_EXTENSION_INCHES, WALL_PRIME_ROTATION_DEGREES);
                } else if (gamepad2.dpad_down) {
                    arm.setTargetPositionPolar(HOME_EXTENSION_INCHES, HOME_ROTATION_DEGREES); // Home
                } else {
                    double extensionInput = -gamepad2.right_stick_y;
                    double rotationInput = gamepad2.left_stick_y;

                    arm.setManualInputs(extensionInput, rotationInput);
                }
                break;
            case SUB:
                if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                    arm.setTargetPositionPolar(SUB_EXTENSION_INCHES, SUB_ROTATION_DEGREES);
                    teleOpState = TeleOpState.NORMAL;
                    break;
                }

                if (gamepad2.right_bumper) {
                    intake.open();
                } else if (gamepad2.left_bumper) {
                    intake.close();
                }

                double extensionInput = -gamepad1.left_stick_y;
                double rotationInput = gamepad1.right_stick_y;

                arm.setManualInputs(extensionInput, rotationInput);

                break;
        }

        arm.update();
    }

    private void drive() {
        double drive = -gamepad1.left_stick_y;
        drive *= Math.abs(drive);
        double strafe = -gamepad1.left_stick_x;
        strafe *= Math.abs(strafe);
        double turn = -gamepad1.right_stick_x;
        turn *= Math.abs(turn);
        mecanumDrive.setTeleOpMovementVectors(drive, strafe, turn, true);
    }

    private enum TeleOpState {
        SUB,
        NORMAL
    }
}
