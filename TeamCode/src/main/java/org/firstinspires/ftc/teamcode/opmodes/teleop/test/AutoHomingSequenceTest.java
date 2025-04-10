package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.hardware.subsystems.arm.Arm;

@Autonomous(group = "Test")
public final class AutoHomingSequenceTest extends OpMode {
    private Arm arm;

    @Override public void init() {
        arm = new Arm(hardwareMap, OpModeMeta.Flavor.AUTONOMOUS);
    }

    @Override public void loop() {
        arm.update();
    }
}
