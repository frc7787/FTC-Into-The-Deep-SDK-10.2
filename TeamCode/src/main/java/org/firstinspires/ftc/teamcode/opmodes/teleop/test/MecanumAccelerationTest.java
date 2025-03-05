package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedropathing.constants.LocalizerConstants;
import org.firstinspires.ftc.teamcode.pedropathing.constants.PathFollowingConstants;
import org.firstinspires.ftc.teamcode.DataLogger;

@TeleOp(group = "Test")
public final class MecanumAccelerationTest extends OpMode {
    private Follower driveBase;
    private ElapsedTime elapsedTime;

    private boolean fileSaved, initialized;

    private DataLogger dataLogger;

    @Override public void init() {
        driveBase = new Follower(hardwareMap, PathFollowingConstants.class, LocalizerConstants.class);

        elapsedTime = new ElapsedTime();
        fileSaved = false;
        initialized = false;

        dataLogger = new DataLogger();
        dataLogger.setHeader("Timer (Seconds),Velocity (Inches / Second)");
    }

    @Override public void start() {
        elapsedTime.reset();
    }

    @Override public void loop() {
        if (!initialized) {
            initialized = true;
            driveBase.setTeleOpMovementVectors(1.0, 0.0, 0.0);
        }

        double seconds = elapsedTime.seconds();
        double velocity = driveBase.getVelocityMagnitude();

        if (seconds > 1.2) driveBase.setTeleOpMovementVectors(0.0, 0.0, 0.0);

        if (seconds > 1.2 && velocity == 0 && !fileSaved) {
            if (!dataLogger.save("MecanumVelocityLog")) {
                telemetry.addLine("Failed To Save File");
            }
        }

        dataLogger.log(seconds + "," + velocity);
    }
}