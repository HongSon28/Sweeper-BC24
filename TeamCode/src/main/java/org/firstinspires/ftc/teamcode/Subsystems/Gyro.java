package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.DriveConstants;

public class Gyro {
    public IMU imu;

    private double last_x_accel = -1;
    private double last_y_accel = -1;

    public Gyro(HardwareMap hardwareMap) {
        // Init IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // Set direction
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);
        imu.resetYaw();
    }
}
