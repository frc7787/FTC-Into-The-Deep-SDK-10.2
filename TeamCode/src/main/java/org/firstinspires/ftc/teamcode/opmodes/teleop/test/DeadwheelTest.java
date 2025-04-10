package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

@TeleOp(group = "Test")
public final class DeadwheelTest extends OpMode {
    private List<DcMotor> motors;

    @Override public void init() {
       motors = List.of(
               hardwareMap.get(DcMotor.class, "leaderExtensionMotor"),
               hardwareMap.get(DcMotor.class, "followerExtensionMotorOne"),
               hardwareMap.get(DcMotor.class, "followerExtensionMotorTwo")
       );
       motors.forEach(motor -> {
           motor.setDirection(DcMotorSimple.Direction.REVERSE);
           motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
           motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       });
    }

    @Override public void loop() {
        double power = -gamepad1.left_stick_y;
        motors.forEach(motor -> motor.setPower(power));
        telemetry.addData("Position", motors.get(0).getCurrentPosition());
    }
}
