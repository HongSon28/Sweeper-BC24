package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Map.DistanceData;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.DistSensor;
import org.firstinspires.ftc.teamcode.Subsystems.ServoAngle;
import static org.firstinspires.ftc.teamcode.Constants.GRID.*;
import static org.firstinspires.ftc.teamcode.Constants.DISTANCE_SENSOR.*;

import java.util.Vector;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Auto Collect Drive")
public class AutoCollectData extends LinearOpMode {

    boolean[][] isVisited = new boolean[50][50];
    Vector2d[][] lastPos = new Vector2d[50][50];
    int cx = 0, cy = 0;
    private SampleTankDrive tankDrive;
    private DistSensor distSensor;
    private ServoAngle servo;
    private double currentAngle = 0;
    public Vector <DistanceData> dataStorage;
    @Override
    public void runOpMode() {
        tankDrive = new SampleTankDrive(hardwareMap);
        distSensor = new DistSensor(hardwareMap);
        servo = new ServoAngle(hardwareMap);

        dataStorage = new Vector<DistanceData>();


        waitForStart();

        while (opModeIsActive()) {
            if (!isVisited[cx][cy]) {
                updateData();
                isVisited[cx][cy] = true;
            }
            runNextBox();
            tankDrive.updatePoseEstimate();

            servo.setAngle(currentAngle);

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
        telemetry.addData("Clear Check: ",distSensor.getRawDist());
        telemetry.update();
        return distSensor.getRawDist() >= CLEAR_THRESHOLD;
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

    public void goBackward() {
        Trajectory trajectory = tankDrive.trajectoryBuilder(new Pose2d()).back(GRID_SIZE).build();
        tankDrive.followTrajectory(trajectory);
        tankDrive.waitForIdle();
        tankDrive.updatePoseEstimate();
    }


    public void Turn(double angle) {
        tankDrive.turn(Math.toRadians(angle));
        tankDrive.waitForIdle();
        tankDrive.updatePoseEstimate();
    }

    public void runBack() {
        int x = (int) lastPos[cx][cy].getX();
        int y = (int) lastPos[cx][cy].getY();
        //Go back
        if (x == cx - 1 && y == cy) {
            goBackward();
            cx--;
        }

        //Go front
        if (x == cx + 1 && y == cy) {
            goForward();
            cx++;
        }

        //Go left
        if (x == cx && y == cy - 1) {
            Turn(90);
            goForward();
            Turn(-90);
            cy--;
        }

        //Go right
        if (x == cx && y == cy + 1) {
            Turn(-90);
            goForward();
            Turn(+90);
            cy++;
        }
    }

    public void runNextBox() {
        //Go front
        int tx = cx + 1;
        int ty = cy;
        if (check(tx,ty)) {
            if (isClear()) {
                lastPos[tx][ty] = new Vector2d(cx,cy);
                cx = tx;
                cy = ty;
                isVisited[cx][cy] = true;

                goForward();

                updateData();
                return;
            }
        }

        //Go back
        tx = cx - 1;
        ty = cy;
        if (check(tx,ty)) {
            Turn(180);
            if (isClear()) {
                lastPos[tx][ty] = new Vector2d(cx,cy);
                cx = tx;
                cy = ty;
                isVisited[cx][cy] = true;

                goForward();

                updateData();

                Turn(180);
                return;
            }
            Turn(180);
        }

        //Go Left
        tx = cx;
        ty = cy - 1;
        if (check(tx,ty)) {
            Turn(90);
            if (isClear()) {
                lastPos[tx][ty] = new Vector2d(cx,cy);
                cx = tx;
                cy = ty;
                isVisited[cx][cy] = true;

                goForward();

                updateData();

                Turn(-90);
                return;
            }
            Turn(-90);
        }

        //Go Right
        tx = cx;
        ty = cy + 1;
        if (check(tx,ty)) {
            Turn(-90);
            if (isClear()) {
                lastPos[tx][ty] = new Vector2d(cx, cy);
                cx = tx;
                cy = ty;
                isVisited[cx][cy] = true;

                goForward();

                updateData();

                Turn(90);
                return;
            }
            Turn(90);
        }
        if (cx == 0 && cy == 0) return;

        runBack();
    }

    public void updateData() {
        for (int i = 0; i < 8 ; i++) {
            Pose2d currentPose = tankDrive.getPoseEstimate();
            DistanceData currentData = new DistanceData(currentPose, distSensor.getDist(), currentAngle);
            dataStorage.add(currentData);
            Turn(45);
        }
    }
}