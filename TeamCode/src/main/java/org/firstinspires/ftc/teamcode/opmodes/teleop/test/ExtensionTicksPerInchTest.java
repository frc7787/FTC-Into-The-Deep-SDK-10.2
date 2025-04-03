package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import static org.firstinspires.ftc.teamcode.hardware.subsystems.Arm.LEADER_EXTENSION_MOTOR_NAME;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Motor;

@TeleOp(group = "Test")
public class ExtensionTicksPerInchTest extends OpMode {
    private Motor leaderExtensionMotor;

    @Override public void init() {
        leaderExtensionMotor = new Motor(hardwareMap.get(DcMotor.class, LEADER_EXTENSION_MOTOR_NAME));
        leaderExtensionMotor.reset();
    }

    @Override public void loop() {
        telemetry.addData("Position (Ticks)", leaderExtensionMotor.position());
    }
}
