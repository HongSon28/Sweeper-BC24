package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistSensor {
    private DistanceSensor sensorDistance;
    public DistSensor(HardwareMap hardwareMap) {
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
    }

    public double getDist() {
        return sensorDistance.getDistance(DistanceUnit.INCH);
    }
}
