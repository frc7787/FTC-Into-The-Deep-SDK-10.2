package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;

@TeleOp(group = "Test")
public final class IntakeTest extends OpMode {
    private Intake intake;

    @Override public void init() { intake = new Intake(hardwareMap); }

    @Override public void loop() {
        if (gamepad1.left_bumper || gamepad2.left_bumper) {
           intake.open();
        } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
            intake.close();
        } else if (gamepad1.triangle || gamepad2.triangle) {
            intake.neutral();
        }

        if (gamepad1.options || gamepad2.options) {
            displayControls();
        } else {
            telemetry.addLine("Press options to display controls");
            intake.debug(telemetry);
        }
    }

    private void displayControls() {
        telemetry.addLine("----- Controls (Either Gamepad) -----");
        telemetry.addLine("Press left bumper to open intake");
        telemetry.addLine("Press right bumper to close intake");
        telemetry.addLine("Press triangle to move intake to neutral position");
    }
}
