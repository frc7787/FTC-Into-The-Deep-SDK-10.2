package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
public class RotationCurrentTest extends OpMode {
    private DcMotorImplEx rotationMotor;

    private ElapsedTime timer;
    private boolean initialized, fileSaved;

    private ArrayList<String> data;

    @Override public void init() {
        rotationMotor = hardwareMap.get(DcMotorImplEx.class, "rotationMotor");
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        timer = new ElapsedTime();
        initialized = false;
        fileSaved = false;
        data = new ArrayList<>();
    }

    @Override public void loop() {
       if (!initialized) {
           rotationMotor.setPower(1.0);
           timer.reset();
           initialized = true;
       }

        if (timer.seconds() > 1.0) {
            rotationMotor.setPower(0.0);

            if (!fileSaved && rotationMotor.getVelocity(AngleUnit.DEGREES) == 0) {
                saveFile();
                fileSaved = true;
            }
        }

        double extensionCurrentAmps = rotationMotor.getCurrent(CurrentUnit.AMPS);
        double rpm = rotationMotor.getVelocity(AngleUnit.DEGREES) / 6.0;

        data.add(timer.seconds() + "," + extensionCurrentAmps + "," + rpm);
    }

    private void saveFile() {
        String currentDate =
                new SimpleDateFormat("yyyyMMdd", Locale.CANADA).format(new Date());
        String currentTime =
                new SimpleDateFormat("HHmmss", Locale.CANADA).format(new Date());

        String aprilTagLogFileName = "RotationCurrentAndRPMLog" + currentDate + "_" + currentTime + ".txt";

        String pathToAprilTagLogFile
                = "/sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/" + aprilTagLogFileName;

        try {
            File aprilTagLogFile = new File(pathToAprilTagLogFile);
            FileWriter fileWriter = new FileWriter(aprilTagLogFile, true);
            BufferedWriter bufferedWriter = new BufferedWriter(fileWriter);

            bufferedWriter.write("Time (Seconds), Current (Amps), RPM");

            for (String entry: data) {
                bufferedWriter.write(entry + "\n");
            }

            bufferedWriter.close();
        } catch (IOException e) {
            telemetry.addData("Failed to write to file", e);
        }
    }
}
