package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(group = "Test")
public class PotentiometerTest extends OpMode {
    private AnalogInput potentiometer;
    private DcMotor rotationMotor;
    private DigitalChannel frontRotationLimitSwitch, backRotationLimitSwitch;

    @Override public void init() {
        potentiometer = hardwareMap.get(AnalogInput.class, "rotationPotentiometer");
        rotationMotor = hardwareMap.get(DcMotorImplEx.class, "rotationMotor");
        frontRotationLimitSwitch = hardwareMap.get(DigitalChannel.class, "frontRotationLimitSwitch");
        backRotationLimitSwitch = hardwareMap.get(DigitalChannel.class, "backRotationLimitSwitch");
        frontRotationLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        backRotationLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override public void loop() {
        telemetry.addData("Voltage", potentiometer.getVoltage());
        rotationMotor.setPower(-gamepad1.left_stick_y);
        telemetry.addData("Front Limit Switch", !frontRotationLimitSwitch.getState());
        telemetry.addData("Back Limit Switch", !backRotationLimitSwitch.getState());
    }
}
