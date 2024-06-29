package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Constants {
    public static class DISTANCE_SENSOR {
        public static final Pose2d DS_LOCATION = new Pose2d(0, 1, 0);

        public static final double KALMAN_Q = 1.5;
        public static final double KALMAN_R = 3;
        public static final int KALMAN_N = 3;
        public static final int DISTANCE_THRESHOLD = 75;
        public static final double CLEAR_THRESHOLD = 30;
    }
    public static class GRID {
        public static final double GRID_SIZE = 20;
    }
}
