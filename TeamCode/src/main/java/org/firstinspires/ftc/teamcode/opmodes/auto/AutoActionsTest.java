package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.roadrunner.actions.HomeArmAction;
import org.firstinspires.ftc.teamcode.roadrunner.actions.RetractArmAction;

@Autonomous(group = "Test")
public class AutoActionsTest extends LinearOpMode {

    @Override public void runOpMode() {
        Arm arm = new Arm(hardwareMap);

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new HomeArmAction(arm),
                        new RetractArmAction(arm, 1.0, 0.9)
                )
        );
    }
}
