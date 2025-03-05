package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import static org.firstinspires.ftc.teamcode.hardware.subsystems.Arm.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import org.firstinspires.ftc.teamcode.DataLogger;
import org.firstinspires.ftc.teamcode.hardware.MotorGroup;
import org.firstinspires.ftc.teamcode.hardware.Motor.AngularVelocityUnit;

@TeleOp(group = "Test")
public final class ExtensionCurrentTest extends OpMode {
    private MotorGroup extensionMotorGroup;

    private ElapsedTime timer;
    private boolean initialized, fileSaved;

    private DataLogger dataLogger;

    @Override public void init() {
        extensionMotorGroup = new MotorGroup(
                hardwareMap.get(DcMotor.class, LEADER_EXTENSION_MOTOR_NAME),
                hardwareMap.get(DcMotor.class, FOLLOWER_EXTENSION_MOTOR_ONE_NAME),
                hardwareMap.get(DcMotor.class, FOLLOWER_EXTENSION_MOTOR_TWO_NAME)
        );
        extensionMotorGroup.setDirection(DcMotorSimple.Direction.REVERSE);

        timer = new ElapsedTime();
        initialized = false;
        fileSaved = false;

        dataLogger = new DataLogger();
        dataLogger.setHeader("Time (Seconds),Current (Amps),RPM");
    }

    @Override public void loop() {
        if (!initialized) {
            extensionMotorGroup.setPower(1.0);
            timer.reset();
            initialized = true;
       }

        double seconds = timer.seconds();
        double amps = extensionMotorGroup.getCurrentSum(CurrentUnit.AMPS);
        double rpm = extensionMotorGroup.getVelocity(AngularVelocityUnit.RPM);

        if (seconds > 1.0) extensionMotorGroup.setPower(0.0);

        if (seconds > 1.0 && rpm == 0.0 && !fileSaved) {
            if (!dataLogger.save("ExtensionCurrentAndRPMLog")) {
                telemetry.addLine("Failed To Save File");
            }
            fileSaved = true;
        }

        dataLogger.log(seconds + "," + amps + "," + rpm);
    }
}
