package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;

@TeleOp(group = "Test")
public class ArmTest extends OpMode {
    private Arm arm;

    @Override public void init() {
        arm = new Arm(hardwareMap, OpModeMeta.Flavor.TELEOP);
    }

    @Override public void loop() {
        if (gamepad1.cross || gamepad2.cross) {
            arm.setTargetPositionPolar(25.0, 45.0);
        } else if (gamepad1.circle || gamepad2.circle) {
            arm.setTargetPositionPolar(35.0, 70.0);
        }

        arm.update();
        arm.globalDebug(telemetry);
        arm.positionDebug(telemetry);
    }
}
