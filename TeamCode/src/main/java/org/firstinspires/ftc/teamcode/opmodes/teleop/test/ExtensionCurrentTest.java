package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Locale;

@TeleOp(group = "Test")
public final class ExtensionCurrentTest extends OpMode {
    private DcMotorImplEx extensionMotorOne, extensionMotorTwo;

    private ElapsedTime timer;
    private boolean initialized, fileSaved;

    private ArrayList<String> data;

    @Override public void init() {
        extensionMotorOne = hardwareMap.get(DcMotorImplEx.class, "extensionMotorOne");
        extensionMotorTwo = hardwareMap.get(DcMotorImplEx.class, "extensionMotorTwo");
        extensionMotorOne.setDirection(DcMotorSimple.Direction.REVERSE);
        extensionMotorTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        extensionMotorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        timer = new ElapsedTime();
        initialized = false;
        fileSaved = false;
        data = new ArrayList<>();
    }

    @Override public void loop() {
       if (!initialized) {
           extensionMotorOne.setPower(-1.0);
           extensionMotorTwo.setPower(-1.0);
           timer.reset();
           initialized = true;
       }

        if (timer.seconds() > 1.0) {
            extensionMotorOne.setPower(0.0);
            extensionMotorTwo.setPower(0.0);

            if (!fileSaved && extensionMotorOne.getVelocity() == 0) {
                saveFile();
                fileSaved = true;
            }
        }

        double extensionCurrentAmps = extensionMotorOne.getCurrent(CurrentUnit.AMPS) +
                extensionMotorTwo.getCurrent(CurrentUnit.AMPS);
        double ticksPerSecond = extensionMotorOne.getVelocity();

        data.add(timer.seconds() + "," + extensionCurrentAmps + "," + ticksPerSecond);
    }

    private void saveFile() {
        String currentDate =
                new SimpleDateFormat("yyyyMMdd", Locale.CANADA).format(new Date());
        String currentTime =
                new SimpleDateFormat("HHmmss", Locale.CANADA).format(new Date());

        String logFileName = "ExtensionCurrentAndRPMLog" + currentDate + "_" + currentTime + ".txt";

        String pathToAprilTagLogFile
                = "/sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/" + logFileName;

        try {
            File aprilTagLogFile = new File(pathToAprilTagLogFile);
            FileWriter fileWriter = new FileWriter(aprilTagLogFile, true);
            BufferedWriter bufferedWriter = new BufferedWriter(fileWriter);

            bufferedWriter.write("Time (Seconds),Current (Amps), Ticks / Second\n");

            for (String entry: data) {
                bufferedWriter.write(entry + "\n");
            }

            bufferedWriter.close();
        } catch (IOException e) {
            telemetry.addData("Failed to write to file", e);
        }
    }
}
