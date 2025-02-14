package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "Test")
public class OpticalOdometryTest extends OpMode {
    private SparkFunOTOS opticalOdometry;

    @Override public void init() {
        opticalOdometry = hardwareMap.get(SparkFunOTOS.class, "opticalOdometry");
        opticalOdometry.calibrateImu(255, true);
        opticalOdometry.resetTracking();
        opticalOdometry.initialize();
    }

    @Override public void loop() {
        SparkFunOTOS.Pose2D position = opticalOdometry.getPosition();
        telemetry.addData("H", position.h);
        telemetry.addData("X", position.x);
        telemetry.addData("Y", position.y);
    }
}
