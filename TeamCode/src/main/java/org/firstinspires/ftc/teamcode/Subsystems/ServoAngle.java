package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoAngle {
    private Servo servo;
    private double offset = 0;
    public ServoAngle(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setPosition(offset);
    }

    public void setAngle(double angle) {
        servo.setPosition(offset + angle / 270);
    }
}
