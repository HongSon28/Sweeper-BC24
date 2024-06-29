package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Map.DistanceData;
import org.firstinspires.ftc.teamcode.Map.MapDrawer;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.DistSensor;
import org.firstinspires.ftc.teamcode.Subsystems.ServoAngle;

import java.util.Vector;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "Manual scan")
public class ManualDrive extends LinearOpMode {

    private SampleTankDrive tankDrive;
    private DistSensor distSensor;
    //private ServoAngle servo;
    private boolean manualDrive = true;
    private boolean updateRequired = true;
    private double currentAngle = 0;
    public Vector <DistanceData> dataStorage;

    private FtcDashboard dashboard;
    private MapDrawer mapDrawer;

    private ElapsedTime timer = new ElapsedTime();
    private double last_time = 0;

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        mapDrawer = new MapDrawer(dashboard, 8, 15, 1);
        dataStorage = new Vector<DistanceData>();

        tankDrive = new SampleTankDrive(hardwareMap);
        distSensor = new DistSensor(hardwareMap);
        //servo = new ServoAngle(hardwareMap);

        waitForStart();
        mapDrawer.init(this);
        timer.reset();
        //mapDrawer.drawState(0, 0, 0, 0);

        boolean prev_state = false;

        while (opModeIsActive()) {
            if (manualDrive) {
                //tankDrive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, 0, gamepad1.left_stick_x));
                tankDrive.setMotorPowers(-gamepad1.left_stick_y * 0.7, -gamepad1.right_stick_y * 0.7);
                tankDrive.updatePoseEstimate();
                if (gamepad1.right_bumper) currentAngle++;
                else if (gamepad1.left_bumper) currentAngle--;

                //servo.setAngle(currentAngle);
                boolean curr_state = gamepad1.right_bumper;
                double curr_time = timer.now(TimeUnit.MILLISECONDS);
                if (updateRequired && curr_time - last_time >= 500) {
                    updateData();
                    last_time = curr_time;
                }

                prev_state = curr_state;
                if (gamepad1.triangle) updateRequired = false;
                else if (gamepad1.square) updateRequired = true;
            }

            Pose2d currentPose = tankDrive.getPoseEstimate();
            telemetry.addData("Current X: ", currentPose.getX());
            telemetry.addData("Current Y: ", currentPose.getY());
            telemetry.addData("Current Yaw (rad): ", mapDrawer.angleWrap(currentPose.getHeading()));
            telemetry.addData("Current Yaw (deg): ", Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("Current Dist: ", distSensor.getDist());
            telemetry.addData("Current Angle: ", currentAngle);
            telemetry.update();
        }
    }

    public void updateData() {
        Pose2d currentPose = tankDrive.getPoseEstimate();
        double distance = distSensor.getDist();
        if (distance == -1) {
            return;
        }

        DistanceData currentData = new DistanceData(currentPose, distance, currentAngle);
        dataStorage.add(currentData);

        Vector2d rotatedVec = currentPose.vec().rotated(0); // Rotate from Roadrunner's system to our system
        mapDrawer.drawState(rotatedVec.getX(), rotatedVec.getY(), mapDrawer.angleWrap(currentPose.getHeading()), distance);
    }
}
