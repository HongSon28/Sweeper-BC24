package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Map.DistanceData;
import org.firstinspires.ftc.teamcode.Map.MapDrawer;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.DistSensor;
import org.firstinspires.ftc.teamcode.Subsystems.ServoAngle;
import static org.firstinspires.ftc.teamcode.Constants.GRID.*;
import static org.firstinspires.ftc.teamcode.Constants.DISTANCE_SENSOR.*;

import java.util.Vector;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "Auto Collect Drive")
public class AutoCollectData extends LinearOpMode {

    boolean[][] isVisited = new boolean[50][50];
    Vector2d[][] lastPos = new Vector2d[50][50];
    int cx = 20, cy = 20;
    private SampleTankDrive tankDrive;
    private DistSensor distSensor;
    private ServoAngle servo;
    public static double currentAngle = 226;
    public Vector <DistanceData> dataStorage;
    private MapDrawer mapDrawer;
    private FtcDashboard dashboard;
    @Override
    public void runOpMode() {
        tankDrive = new SampleTankDrive(hardwareMap);
        distSensor = new DistSensor(hardwareMap);
        servo = new ServoAngle(hardwareMap);

        dataStorage = new Vector<DistanceData>();

        dashboard = FtcDashboard.getInstance();
        mapDrawer = new MapDrawer(dashboard, 8, 15, 1);

        waitForStart();

        while (opModeIsActive()) {
            servo.setAngle(currentAngle);
            if (!isVisited[cx][cy]) {
                updateData();
                isVisited[cx][cy] = true;
            }
            runNextBox();
            tankDrive.updatePoseEstimate();

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
        sleep(600);
        double temp = distSensor.getRawDist();
        telemetry.addData("Clear Check: ",temp);
        telemetry.update();
        return temp >= CLEAR_THRESHOLD;
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

    public void goTo(int x, int y) {
        Trajectory trajectory = tankDrive.trajectoryBuilder(tankDrive.getPoseEstimate()).splineTo(new Vector2d((cx - 20) * GRID_SIZE, (cy - 20) * GRID_SIZE),Math.toRadians(0)).build();
        tankDrive.followTrajectory(trajectory);
        //tankDrive.waitForIdle();
        //tankDrive.updatePoseEstimate();
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
            //goBackward();
            cx--;
            goTo(cx,cy);
        }

        //Go front
        if (x == cx + 1 && y == cy) {
            //goForward();
            cx++;
            goTo(cx,cy);
        }

        //Go left
        if (x == cx && y == cy - 1) {
            //Turn(90);
            //goForward();
            //Turn(-90);
            cy--;
            goTo(cx,cy);
        }

        //Go right
        if (x == cx && y == cy + 1) {
            //Turn(-90);
            //goForward();
            //Turn(+90);
            cy++;
            goTo(cx,cy);
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

                //goForward();
                goTo(cx,cy);

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
                Turn(90);
                Turn(90);
                lastPos[tx][ty] = new Vector2d(cx,cy);
                cx = tx;
                cy = ty;
                isVisited[cx][cy] = true;

                //goForward();
                goTo(cx,cy);

                updateData();

                //Turn(180);
                return;
            }
            Turn(180);
        }

        //Go Left
        tx = cx;
        ty = cy - 1;
        if (check(tx,ty)) {
            Turn(-90);
            if (isClear()) {
                lastPos[tx][ty] = new Vector2d(cx,cy);
                cx = tx;
                cy = ty;
                isVisited[cx][cy] = true;

                //goForward();
                goTo(cx,cy);

                updateData();

                //Turn(-90);
                return;
            }
            Turn(90);
        }

        //Go Right
        tx = cx;
        ty = cy + 1;
        if (check(tx,ty)) {
            Turn(90);
            if (isClear()) {
                lastPos[tx][ty] = new Vector2d(cx, cy);
                cx = tx;
                cy = ty;
                isVisited[cx][cy] = true;

                //goForward();
                goTo(cx,cy);

                updateData();

                //Turn(90);
                return;
            }
            Turn(-90);
        }
        if (cx == 20 && cy == 20) return;

        runBack();
    }

    public void updateData() {
        Pose2d currentPose;
        for (int i = 0; i < 8 ; i++) {
            sleep(500);
            currentPose = tankDrive.getPoseEstimate();
            DistanceData currentData = new DistanceData(currentPose, distSensor.getDist(), currentAngle);
            dataStorage.add(currentData);
            mapDrawer.drawState(currentPose.getX(), currentPose.getY(), mapDrawer.angleWrap(currentPose.getHeading()), distSensor.getDist());
            Turn(45);
        }

        currentPose = tankDrive.getPoseEstimate();
        Turn(Math.toDegrees(-currentPose.getHeading()));
    }
}
