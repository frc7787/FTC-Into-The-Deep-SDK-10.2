package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import static org.firstinspires.ftc.teamcode.hardware.subsystems.Arm.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.PIDController;

@TeleOp(group = "Test")
public class RotationPositionTest extends OpMode {
    private Motor rotationMotor;
    private PIDController rotationController;

    int rotationTargetPosition;

    @Override public void init() {
        rotationMotor = new Motor(hardwareMap.get(DcMotor.class, ROTATION_MOTOR_NAME));
        rotationMotor.reset();
        rotationController = new PIDController(ROTATION_KP, ROTATION_KI, ROTATION_KD);
        rotationTargetPosition = 0;
    }

    @Override public void loop() {
        if (gamepad1.dpad_up) {
            rotationTargetPosition = 1513;
        } else if (gamepad1.dpad_down) {
            rotationTargetPosition = 3026;
        }

        int position = rotationMotor.position();
        double power
                = rotationController.calculate(position, rotationTargetPosition);
        rotationMotor.setPower(power);
        telemetry.addData("Position", position);
    }
}
