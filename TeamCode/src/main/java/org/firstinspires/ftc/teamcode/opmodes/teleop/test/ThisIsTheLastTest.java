package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group = "Test")
public class ThisIsTheLastTest extends OpMode {
    private DcMotor rotationMotor;

    @Override public void init() {
       rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
    }

    @Override public void loop() {
        rotationMotor.setPower(gamepad1.left_stick_x);
    }
}
