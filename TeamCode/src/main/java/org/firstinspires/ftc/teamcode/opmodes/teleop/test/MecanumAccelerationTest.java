package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.io.*;
import java.text.SimpleDateFormat;
import java.util.*;

@TeleOp
public final class MecanumAccelerationTest extends OpMode {
    private MecanumDrive driveBase;
    private ElapsedTime elapsedTime;

    private ArrayList<String> timeAndVelocityPairs;

    private boolean fileSaved, initialized;

    @Override public void init() {
        driveBase = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));

        elapsedTime = new ElapsedTime();
        timeAndVelocityPairs = new ArrayList<>();
        fileSaved = false;
        initialized = false;
    }

    @Override public void start() {
        elapsedTime.reset();
    }

    @Override public void loop() {
        double velocity = driveBase.localizer.par.getPositionAndVelocity().velocity;
        double time = elapsedTime.seconds();

        if (!initialized) {
            initialized = true;
            driveBase.robotCentric(1.0, 0, 0);
        }

        if (elapsedTime.seconds() > 1.2) {
            driveBase.robotCentric(0,0,0);
            if (!fileSaved && velocity == 0.0) {
                fileSaved = true;
                saveFile();
            }
        }

        timeAndVelocityPairs.add(velocity + "," + time);
    }

    private void saveFile() {
        String currentDate =
                new SimpleDateFormat("yyyyMMdd", Locale.CANADA).format(new Date());
        String currentTime =
                new SimpleDateFormat("HHmmss", Locale.CANADA).format(new Date());

        String aprilTagLogFileName = "MecanumVelocityLog" + currentDate + "_" + currentTime + ".txt";

        String pathToAprilTagLogFile
                = "/sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/" + aprilTagLogFileName;

        try {
            File aprilTagLogFile = new File(pathToAprilTagLogFile);
            FileWriter fileWriter = new FileWriter(aprilTagLogFile, true);
            BufferedWriter bufferedWriter = new BufferedWriter(fileWriter);

            bufferedWriter.write("Ticks / Second,Seconds\n");

            for (String timeAndVelocityPair : timeAndVelocityPairs) {
                bufferedWriter.write(timeAndVelocityPair + "\n");
            }

            bufferedWriter.close();
        } catch (IOException e) {
            telemetry.addData("Failed to write to file", e);
        }
    }

}