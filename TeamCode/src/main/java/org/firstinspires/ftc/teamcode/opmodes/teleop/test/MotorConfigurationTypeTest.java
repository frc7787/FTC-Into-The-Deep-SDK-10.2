package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@TeleOp(group = "Test")
public final class MotorConfigurationTypeTest extends OpMode {
    private DcMotorImplEx testMotor;

    @Override public void init() {
        testMotor = hardwareMap.get(DcMotorImplEx.class, "frontLeftDriveMotor");
    }

    @Override public void loop() {
        MotorConfigurationType motorType = testMotor.getMotorType();

        telemetry.addData("Gearing", motorType.getGearing());
        telemetry.addData("Max RPM fraction", motorType.getAchieveableMaxRPMFraction());
        telemetry.addData("Max RPM", motorType.getMaxRPM());
    }
}
