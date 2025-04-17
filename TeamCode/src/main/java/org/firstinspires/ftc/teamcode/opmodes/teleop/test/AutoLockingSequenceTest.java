package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.hardware.subsystems.arm.Arm;

@Autonomous(group = "Test")
public class AutoLockingSequenceTest extends OpMode {
    private Arm arm;
    private Hanger hanger;

    private int pathState;

    private ElapsedTime lockServoTimer;

    @Override public void init() {
        arm = new Arm(hardwareMap, OpModeMeta.Flavor.TELEOP);
        hanger = new Hanger(hardwareMap);

        lockServoTimer = new ElapsedTime();

        pathState = 0;
    }

    @Override public void loop() {
        switch (pathState) {
            case 0:
                if (arm.state() != Arm.State.HOMING) {
                    arm.setTargetPositionPolar(15.0, 90.0);
                    pathState++;
                }
                arm.update();
                break;
            case 1:
                if (arm.polarCoordinates()[0] > 85.0) {
                    arm.setPower(-0.2, 0.3);
                    lockServoTimer.reset();
                    hanger.lock();
                    pathState++;
                    break;
                }
                arm.update();
                break;
            case 2:
                if (lockServoTimer.milliseconds() > 500) {
                    hanger.releaseLock();
                    lockServoTimer.reset();
                    pathState++;
                }
                break;
            case 3:
                if (lockServoTimer.milliseconds() > 500) {
                    telemetry.addLine("Release the arm!!");
                }
                break;
            default:
                break;
        }
    }
}
