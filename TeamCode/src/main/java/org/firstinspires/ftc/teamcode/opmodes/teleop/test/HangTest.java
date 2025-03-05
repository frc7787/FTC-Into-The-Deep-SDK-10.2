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
        if (gamepad1.cross) {
            hanger.release();
        } else if (gamepad1.dpad_up) {
            telemetry.addLine("Idle!");
            hanger.idle();
        } else if (gamepad1.dpad_down) {
            hanger.prime();
            telemetry.addLine("Primed!");
        }

        hanger.debug(telemetry);
    }
}
