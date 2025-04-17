package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.hardware.subsystems.arm.Arm;

@TeleOp(group = "Test")
public final class HangTest extends OpMode {
    private Hanger hanger;
    private Arm arm;

    @Override public void init() {
        hanger = new Hanger(hardwareMap);
        arm = new Arm(hardwareMap, OpModeMeta.Flavor.TELEOP);
    }

    @Override public void loop() {
        if (gamepad1.cross || gamepad2.cross) {
            hanger.release();
            arm.setManualInputs(-1.0, 0.0);
        } else if (gamepad1.dpad_up || gamepad2.dpad_up) {
            hanger.idle();
        } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
            hanger.prime();
        } else if (gamepad1.dpad_left || gamepad2.dpad_left) {
            hanger.lock();
        }
//        else if (gamepad1.dpad_right) {
//            hanger.lock();
//        }
        else {
            arm.setManualInputs(-gamepad1.right_stick_y, gamepad1.left_stick_y);
        }

        arm.update();

        arm.positionDebug(telemetry);

        if (gamepad1.options || gamepad2.options) {
            displayControls();
        } else {
            telemetry.addLine("Press options on either gamepad to display the controls");
            hanger.debug(telemetry);
        }
    }

    private void displayControls() {
        telemetry.addLine("----- Controls (Either Gamepad) -----");
        telemetry.addLine("Press dpad down to prime hanger");
        telemetry.addLine("Press dpad up to move hanger to idle");
        telemetry.addLine("Press Cross to release the hanger");
        telemetry.addLine("Press Dpad Left to lock the hanger");
    }
}
