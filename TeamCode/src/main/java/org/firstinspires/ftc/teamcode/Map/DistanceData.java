package org.firstinspires.ftc.teamcode.Map;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class DistanceData {
    public Pose2d pose;
    public double dist;
    public double pitchAngle;
    public DistanceData(Pose2d pose, double dist, double pitchAngle) {
        this.pose = pose;
        this.dist = dist;
        this.pitchAngle = pitchAngle;
    }
}
