package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ServoAngle {
    private ServoImplEx servo;
    private double offset = 0;
    public ServoAngle(HardwareMap hardwareMap) {
        servo = (ServoImplEx) hardwareMap.get(Servo.class, "servo");
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        servo.setPosition(offset);
    }

    public void setAngle(double angle) {
        servo.setPosition(offset + angle / 270);
    }
}
