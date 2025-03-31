package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Hanger;

@TeleOp(group = "Test")
public final class HangTest extends OpMode {
    private Hanger hanger;

    @Override public void init() {
        hanger = new Hanger(hardwareMap);
    }

    @Override public void loop() {
        if (gamepad1.cross || gamepad2.cross) {
            hanger.release();
        } else if (gamepad1.dpad_up || gamepad2.dpad_up) {
            hanger.idle();
        } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
            hanger.prime();
        }

        if (gamepad1.options || gamepad2.options) {
            displayControls();
        } else {
            telemetry.addLine("Press options to display controls");
            hanger.debug(telemetry);
        }
    }

    private void displayControls() {
        telemetry.addLine("----- Controls (Either Gamepad) -----");
        telemetry.addLine("Press dpad down to prime hanger");
        telemetry.addLine("Press dpad up to move hanger to idle");
        telemetry.addLine("Press Cross to release the hanger");
    }
}
