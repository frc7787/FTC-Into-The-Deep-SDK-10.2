package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.hardware.subsystems.arm.Arm;

import java.util.ArrayList;
import java.util.List;

@TeleOp(group = "Test")
public class FunctionPerformanceTest extends OpMode {
    private ElapsedTime timer;

    private Arm arm;

    List<Double> functionExecutionTimes;

    @Override public void init() {
        arm = new Arm(hardwareMap, OpModeMeta.Flavor.TELEOP); // This is supposed to be TELEOP

        timer = new ElapsedTime();
        functionExecutionTimes = new ArrayList<>();
    }

    @Override public void start() { timer.reset(); }

    @Override public void loop() {
        double startTime = timer.milliseconds();
        arm.update();
        double endTime = timer.milliseconds();
        functionExecutionTimes.add(endTime - startTime);
        displayAverageExecutionTime();
    }

    private void displayAverageExecutionTime() {
        double sum = 0;
        for(Double d : functionExecutionTimes) { sum += d; }

        telemetry.addData(
                "Average Execution Time Of Update Method",
                sum / functionExecutionTimes.size()
        );
    }
}
