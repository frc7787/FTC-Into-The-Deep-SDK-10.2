package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import static com.qualcomm.robotcore.hardware.DcMotor.*;

import org.firstinspires.ftc.teamcode.hardware.Motor;

@TeleOp(group = "Test")
public final class MotorTest extends OpMode {
    private Motor rotationMotor;

    @Override public void init() {
        rotationMotor = new Motor(hardwareMap.get(DcMotor.class, "rotationMotor"));
    }

    @Override public void loop() {
        rotationMotor.setPower(-gamepad1.left_stick_y);

        if (gamepad1.left_bumper || gamepad2.left_bumper) {
            rotationMotor.setDirection(Direction.REVERSE);
        } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
            rotationMotor.setDirection(Direction.FORWARD);
        }

        if (gamepad1.cross || gamepad2.cross) {
            rotationMotor.setZeroPowerBehaviour(ZeroPowerBehavior.BRAKE);
        } else if (gamepad1.triangle || gamepad2.triangle) {
            rotationMotor.setZeroPowerBehaviour(ZeroPowerBehavior.FLOAT);
        }

        if (gamepad1.options || gamepad2.options) rotationMotor.setPosition(100);

        if (gamepad1.dpad_up || gamepad2.dpad_up) rotationMotor.setPosition(0);

        if (gamepad1.share || gamepad2.share) {
            displayControls();
        } else {
            telemetry.addLine("Press share to display controls");
            rotationMotor.debug(telemetry, "Rotation Motor");
        }
    }

    private void displayControls() {
        telemetry.addLine("----- Controls (Either Gamepad) -----");
        telemetry.addLine("Press left & right bumper to set the motor direction");
        telemetry.addLine("Press cross & triangle to set the motor zero power behavior");
        telemetry.addLine("Press options to set the position of the motor to 100");
        telemetry.addLine("Power the motor with the left stick y");
    }
}
