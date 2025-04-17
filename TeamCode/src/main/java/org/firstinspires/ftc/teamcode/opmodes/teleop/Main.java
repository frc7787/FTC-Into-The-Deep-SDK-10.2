package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.subsystems.arm.Arm;

@TeleOp(group = "$")
@Config
public final class Main extends OpMode {

    // ---------------------------------------------------------------------------------------------
    // Positions

    public static volatile double HIGH_BAR_EXTENSION_INCHES = 35.5;
    public static volatile double HIGH_BAR_ROTATION_DEGREES = 83.0;

    public static volatile double LOW_BAR_EXTENSION_INCHES = 0.0;
    public static volatile double LOW_BAR_ROTATION_DEGREES = 0.0;

    public static volatile double HIGH_BUCKET_EXTENSION_INCHES = 50.0;
    public static volatile double HIGH_BUCKET_ROTATION_DEGREES = 80.0;

    public static volatile double LOW_BUCKET_EXTENSION_INCHES = 0.0;
    public static volatile double LOW_BUCKET_ROTATION_DEGREES = 0.0;

    public static volatile double SUB_EXTENSION_INCHES = 17.5;
    public static volatile double SUB_ROTATION_DEGREES = -5.0;

    public static volatile double HOME_EXTENSION_INCHES = 0.0;
    public static volatile double HOME_ROTATION_DEGREES = 0.0;

    public static volatile double WALL_PRIME_EXTENSION_INCHES = 15.0;
    public static volatile double WALL_PRIME_ROTATION_DEGREES = 92.0;

    public static volatile double WALL_PICKUP_EXTENSION_INCHES = 0.0;

    public static volatile double HANG_EXTENSION_INCHES = 0.0;
    public static volatile double HANG_ROTATION_DEGREES = 0.0;

    private boolean subAtInitialPositionOnce;

    private Arm arm;
    private Intake intake;
    private Hanger hanger;

    private MecanumDrive mecanumDrive;

    private TeleOpState teleOpState;

    private Gamepad currentGamepad1, previousGamepad1;

    @Override public void init() {
        arm = new Arm(hardwareMap, OpModeMeta.Flavor.TELEOP);
        intake = new Intake(hardwareMap);
        hanger = new Hanger(hardwareMap);
        mecanumDrive = new MecanumDrive(hardwareMap);
        teleOpState = TeleOpState.NORMAL;
        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
        subAtInitialPositionOnce = false;
    }

    @Override public void start() { mecanumDrive.resetYaw(); }

    @Override public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        if (gamepad1.options) mecanumDrive.resetYaw();

        drive();

        switch (teleOpState) {
            case NORMAL:
                if (gamepad2.right_bumper) {
                    intake.open();
                } else if (gamepad2.left_bumper) {
                    intake.close();
                }

                if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                    arm.setTargetPositionPolar(SUB_EXTENSION_INCHES, SUB_ROTATION_DEGREES);
                    teleOpState = TeleOpState.SUB;
                    break;
                } else if (gamepad2.circle) {
                    arm.setTargetPositionPolar(HIGH_BAR_EXTENSION_INCHES, HIGH_BAR_ROTATION_DEGREES);
                } else if (gamepad2.triangle) {
                    arm.setTargetPositionPolar(HIGH_BUCKET_EXTENSION_INCHES, HIGH_BUCKET_ROTATION_DEGREES);
                } else if (gamepad2.square) {
                    arm.setTargetPositionPolar(WALL_PRIME_EXTENSION_INCHES, WALL_PRIME_ROTATION_DEGREES);
                } else if (gamepad2.dpad_down) {
                    arm.setTargetPositionPolar(HOME_EXTENSION_INCHES, HOME_ROTATION_DEGREES);
                } else {
                    double extensionInput = -gamepad2.right_stick_y;
                    double rotationInput = gamepad2.left_stick_y;

                    if (extensionInput != 0.0 || rotationInput != 0.0) {
                        arm.setManualInputs(extensionInput, rotationInput);
                    }
                }
                break;
            case SUB:
                if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                    arm.setTargetPositionPolar(SUB_EXTENSION_INCHES, SUB_ROTATION_DEGREES);
                    intake.open();
                    teleOpState = TeleOpState.NORMAL;
                    break;
                }

                if (gamepad1.triangle) {
                    intake.open();
                } else if (gamepad1.circle) {
                    intake.close();
                }

                double extensionInput = gamepad1.left_trigger - gamepad1.right_trigger;
                double rotationInput = -gamepad1.right_stick_y;

                if (extensionInput != 0.0 || rotationInput != 0.0) {
                    arm.setManualInputs(extensionInput, rotationInput);
                }


                break;
        }

        if (gamepad2.dpad_right) {
            hanger.lock();
        } else if (gamepad2.dpad_left) {
            hanger.releaseLock();
        }

//        if (gamepad1.options) {
//            hanger.prime();
//        } else if (gamepad1.share && gamepad2.share) {
//            arm.setManualInputs(-1.0, 0.0);
//            hanger.release();
//        }

        arm.update();
    }

    private void drive() {
        double drive = -gamepad1.left_stick_y;
        drive *= Math.abs(drive);
        double strafe = gamepad1.left_stick_x;
        strafe *= Math.abs(strafe);
        double turn = gamepad1.right_stick_x;
        turn *= Math.abs(turn);
        mecanumDrive.drive(drive, strafe, turn);
    }

    private enum TeleOpState {
        SUB,
        NORMAL
    }
}
