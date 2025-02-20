package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.arm.Arm;

@TeleOp(group = "Test")
public class ArmPositionTest extends OpMode {
    private Arm arm;

    @Override public void init() {
       arm = new Arm(hardwareMap);
    }

    @Override public void loop() {
        if (gamepad2.cross) {
            arm.setTargetPositionPolar(10.0, 45.0);
        } else if (gamepad2.circle) {
            arm.setTargetPositionPolar(10.0, 70.0);
        }

        arm.positionDebug(telemetry);
        arm.globalDebug(telemetry);

        arm.update();
    }
}
