package org.firstinspires.ftc.teamcode.Map;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public class Logger {
    private String log_base_path = "/sdcard/FIRST/";

    private File out_file;
    private File in_file;
    private FileWriter out_w;
    private FileWriter in_w;

    private boolean enable_in_log;

    SimpleDateFormat sdf = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.getDefault());

    public Logger(boolean _enable_in_log) {
        enable_in_log = _enable_in_log;
        String dateTime = sdf.format(new Date());

        try {
            out_file = new File(log_base_path + "out_" + dateTime + ".txt");
            out_file.createNewFile();
            out_w = new FileWriter(out_file, false);
        } catch (IOException e) {
            // Do literally nothing
        }

        if (enable_in_log) {
            try {
                in_file = new File(log_base_path + "in_" + dateTime + ".txt");
                in_file.createNewFile();
                in_w = new FileWriter(in_file, false);
            } catch (IOException e) {
                // Do literally nothing
            }
        }
    }

    public void logO(double x, double y) {
        try {
            out_w.write(Double.toString(x) + "," + Double.toString(y) + "\n");
            out_w.flush();
        } catch (IOException e) {
            // Do literally nothing
        }
    }

    public void logI(double x, double y, double yaw, double distance) {
        logI(x, y, yaw, distance, 0);
    }

    public void logI(double x, double y, double yaw, double distance, double servoAngle) {
        try {
            in_w.write(Double.toString(x) + "," + Double.toString(y) + "," + Double.toString(yaw) + "," + Double.toString(distance) + "," + Double.toString(servoAngle) + "\n");
            in_w.flush();
        } catch (IOException e) {
            // Do literally nothing
        }
    }

    public void close() {
        try {
            out_w.close();
        } catch (IOException e) {
            // Do literally nothing
        }

        if (enable_in_log) {
            try {
                in_w.close();
            } catch (IOException e) {
                // Do literally nothing
            }
        }
    }
}
