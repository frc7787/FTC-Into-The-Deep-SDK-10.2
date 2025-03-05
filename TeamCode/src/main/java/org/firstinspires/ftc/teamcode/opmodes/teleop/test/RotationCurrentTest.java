package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import static org.firstinspires.ftc.teamcode.hardware.subsystems.Arm.ROTATION_MOTOR_NAME;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.DataLogger;
import org.firstinspires.ftc.teamcode.hardware.Motor;

@TeleOp(group = "Test")
public final class RotationCurrentTest extends OpMode {
    private Motor rotationMotor;

    private ElapsedTime timer;
    private boolean initialized, fileSaved;

    private DataLogger dataLogger;

    @Override public void init() {
        rotationMotor = new Motor(hardwareMap.get(DcMotorImplEx.class, ROTATION_MOTOR_NAME));
        timer = new ElapsedTime();
        initialized = false;
        fileSaved = false;
        dataLogger = new DataLogger();
        dataLogger.setHeader("Time (Seconds),Current (AMPS),RPM");
    }

    @Override public void loop() {
       if (!initialized) {
           rotationMotor.setPower(1.0);
           timer.reset();
           initialized = true;
       }

       double seconds = timer.seconds();
       double rpm = rotationMotor.velocity(Motor.AngularVelocityUnit.RPM);
       double amps = rotationMotor.current(CurrentUnit.AMPS);

       if (seconds > 1.0) rotationMotor.setPower(0.0);

       if (seconds > 1.0 && rpm == 0 && !fileSaved) {
           if (!dataLogger.save("CurrentAndRPMLog")) {
               telemetry.addLine("Failed To Save File.");
           }
           fileSaved = true;
       }

        dataLogger.log(timer.seconds() + "," + rpm + "," + amps);
    }
}
