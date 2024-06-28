package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DistanceSensor {
    private DistanceSensor sensorDistance;
    public DistanceSensor(HardwareMap hardwareMap) {
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
    }

    public double getDist() {
        return sensorDistance.getDist();
    }
}
