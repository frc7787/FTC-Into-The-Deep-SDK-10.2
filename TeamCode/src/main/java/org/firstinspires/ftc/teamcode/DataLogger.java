package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Locale;

public final class DataLogger {
    private final ArrayList<String> data;
    private String header;

    public DataLogger() {
        data = new ArrayList<>();
        header = "";
    }

    /**
     * Sets the first header, or first line, of the data file
     * @param header The string to set the header as
     */
    public void setHeader(@NonNull String header) {
        this.header = header;
    }

    /**
     * Adds an entry to the log
     * @param entry The entry to add
     */
    public void log(@NonNull String entry) {
        data.add(entry);
    }

    /**
     * Saves the logged data to the file path specified in the constructor
     * @return True if the save was successful, false otherwise
     */
    public boolean save(@NonNull String logFileName) {
        String currentDate =
                new SimpleDateFormat("yyyyMMdd", Locale.CANADA).format(new Date());
        String currentTime =
                new SimpleDateFormat("HHmmss", Locale.CANADA).format(new Date());
        @SuppressLint("SdCardPath")
        String pathToLogFile
                = "/sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/" + logFileName + "_"
                + currentDate + "_" + currentTime + ".txt";
        File logFile = new File(pathToLogFile);

        try (BufferedWriter logWriter = new BufferedWriter(new FileWriter(logFile, true))) {
            logWriter.write(header);
            for (String entry: data) { logWriter.write(entry + "\n"); }
        } catch (IOException ignored) { return false; }

        return true;
    }
}
