package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Map.DistanceData;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.DistSensor;
import org.firstinspires.ftc.teamcode.Subsystems.ServoAngle;
import static org.firstinspires.ftc.teamcode.Constants.GRID.*;
import static org.firstinspires.ftc.teamcode.Constants.DISTANCE_SENSOR.*;

import java.util.Vector;

@TeleOp(name = "Manual Drive")
public class AutoCollectData extends LinearOpMode {

    boolean[][] isVisited = new boolean[50][50];
    int cx = 0, cy = 0;
    private SampleTankDrive tankDrive;
    private DistSensor distSensor;
    private ServoAngle servo;
    private boolean updateRequired = true;
    private double currentAngle = 0;
    public Vector <DistanceData> dataStorage;
    @Override
    public void runOpMode() {
        tankDrive = new SampleTankDrive(hardwareMap);
        distSensor = new DistSensor(hardwareMap);
        servo = new ServoAngle(hardwareMap);

        isVisited[0][0] = true;

        waitForStart();

        while (opModeIsActive()) {
            runNextBox();
            tankDrive.updatePoseEstimate();

            servo.setAngle(currentAngle);
            if (updateRequired) updateData();

            Pose2d currentPose = tankDrive.getPoseEstimate();
            telemetry.addData("Current X: ", currentPose.getX());
            telemetry.addData("Current Y: ", currentPose.getY());
            telemetry.addData("Current Yaw: ", currentPose.getHeading());
            telemetry.addData("Current Dist: ", distSensor.getDist());
            telemetry.addData("Current Angle: ", currentAngle);
            telemetry.update();
        }
    }

    boolean isClear() {
        return distSensor.getDist() >= CLEAR_THRESHOLD;
    }
    boolean check(int x, int y) {
        return (x>=0 && y>=0 && x<50 && y<50 && !isVisited[x][y]);
    }
    public void goForward() {
        Trajectory trajectory = tankDrive.trajectoryBuilder(new Pose2d()).forward(GRID_SIZE).build();
        tankDrive.followTrajectory(trajectory);
        tankDrive.waitForIdle();
        tankDrive.updatePoseEstimate();
    }

    public void Turn(double angle) {
        tankDrive.turn(Math.toRadians(angle));
        tankDrive.waitForIdle();
        tankDrive.updatePoseEstimate();
    }
    public void runNextBox() {
        //Go front
        int tx = cx + 1;
        int ty = cy;
        if (check(tx,ty)) {
            if (isClear()) {
                cx = tx;
                cy = ty;
                isVisited[cx][cy] = true;

                goForward();
            }
        }

        //Go back
        tx = cx - 1;
        ty = cy;
        if (check(tx,ty)) {
            Turn(180);
            if (isClear()) {
                cx = tx;
                cy = ty;
                isVisited[cx][cy] = true;

                goForward();
            }
            Turn(180);
        }

        //Go Left
        tx = cx;
        ty = cy - 1;
        if (check(tx,ty)) {
            Turn(90);
            if (isClear()) {
                cx = tx;
                cy = ty;
                isVisited[cx][cy] = true;

                goForward();
            }
            Turn(-90);
        }

        //Go Right
        tx = cx;
        ty = cy + 1;
        if (check(tx,ty)) {
            Turn(-90);
            if (isClear()) {
                cx = tx;
                cy = ty;
                isVisited[cx][cy] = true;

                goForward();
            }
            Turn(90);
        }
    }

    public void updateData() {
        Pose2d currentPose = tankDrive.getPoseEstimate();
        DistanceData currentData = new DistanceData(currentPose, distSensor.getDist(), currentAngle);
        dataStorage.add(currentData);
    }
}
