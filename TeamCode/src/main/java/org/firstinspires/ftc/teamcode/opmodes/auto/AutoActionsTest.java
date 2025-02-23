package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.roadrunner.actions.HomeArmAction;
import org.firstinspires.ftc.teamcode.roadrunner.actions.MoveToPositionAction;

@Disabled
@Autonomous(group = "Test")
public class AutoActionsTest extends LinearOpMode {

    @Override public void runOpMode() {
        Arm arm = new Arm(hardwareMap);

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new HomeArmAction(arm),
                        new MoveToPositionAction(arm, 26.0, -6.5, 2.0, telemetry),
                        new SleepAction(3.0),
                        new MoveToPositionAction(arm, 15.0, -6.5, 2.0, telemetry)
//                        new SleepAction(6.0),
//                        new MoveToPositionAction(arm, 0.3, -15.0, 5.0, telemetry)
                )
        );
    }
}
