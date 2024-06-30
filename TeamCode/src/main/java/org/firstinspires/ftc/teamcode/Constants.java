package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class Constants {
    public static class DISTANCE_SENSOR {
        public static final Pose2d DS_LOCATION = new Pose2d(0, 1, 0);

        public static double KALMAN_Q = 1.5;
        public static double KALMAN_R = 3;
        public static int KALMAN_N = 3;
        public static int DISTANCE_THRESHOLD = 75;
        public static final double CLEAR_THRESHOLD = 13.5;
    }
    public static class GRID {
        public static final double GRID_SIZE = 10;
    }

    public static class MAP {
        public static double BOT_CENTER_TO_DISTANCE_SENSOR = 7.08;
    }
}
