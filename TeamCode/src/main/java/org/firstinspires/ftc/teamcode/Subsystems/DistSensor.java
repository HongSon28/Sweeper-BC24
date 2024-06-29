package org.firstinspires.ftc.teamcode.Subsystems;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.teamcode.Constants.DISTANCE_SENSOR.*;

public class DistSensor {
    private DistanceSensor sensorDistance;
    private KalmanFilter filter = new KalmanFilter(KALMAN_Q, KALMAN_R, KALMAN_N);

    public DistSensor(HardwareMap hardwareMap) {
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
    }

    public double getRawDist() {
        return sensorDistance.getDistance(DistanceUnit.INCH);
    }

    public double getDist() {
        double dist = getRawDist();
        if (dist > DISTANCE_THRESHOLD || dist < 0) {
            return -1; // Invalid
        }

        //return filter.estimate(getRawDist());
        return dist;
    }
}
