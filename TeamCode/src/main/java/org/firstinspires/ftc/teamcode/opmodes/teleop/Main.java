package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.arm.Arm;

@TeleOp
public class Main extends OpMode {
    private Arm arm;

    @Override public void init() {
        arm = new Arm(hardwareMap);
    }

    @Override public void loop() {
        arm.update();
    }
}
