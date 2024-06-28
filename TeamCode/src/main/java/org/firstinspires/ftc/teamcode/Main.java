package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.DistSensor;
import org.firstinspires.ftc.teamcode.Subsystems.ServoAngle;

import java.util.Vector;

public class Main extends LinearOpMode {

    private SampleTankDrive tankDrive;
    private DistSensor distSensor;
    private ServoAngle servo;
    private boolean manualDrive = true;
    private boolean updateRequired = true;
    private double currentAngle = 0;
    public Vector <DistanceData> dataStorage;
    @Override
    public void runOpMode() {
        tankDrive = new SampleTankDrive(hardwareMap);
        distSensor = new DistSensor(hardwareMap);
        servo = new ServoAngle(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (manualDrive) {
                tankDrive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, 0, gamepad1.left_stick_x));
                tankDrive.updatePoseEstimate();
                if (gamepad1.right_bumper) currentAngle++;
                else if (gamepad1.left_bumper) currentAngle--;

                servo.setAngle(currentAngle);
                if (updateRequired) updateData();

                if (gamepad1.triangle) updateRequired = false;
                else if (gamepad1.square) updateRequired = true;

                if (gamepad1.cross) manualDrive = false;
            } else {
                if (gamepad1.circle) manualDrive = true;
            }

            Pose2d currentPose = tankDrive.getPoseEstimate();
            telemetry.addData("Current X: ", currentPose.getX());
            telemetry.addData("Current Y: ", currentPose.getY());
            telemetry.addData("Current Yaw: ", currentPose.getHeading());
            telemetry.addData("Current Dist: ", distSensor.getDist());
            telemetry.addData("Current Angle: ", currentAngle);
            telemetry.update();
        }
    }

    public void updateData() {
        Pose2d currentPose = tankDrive.getPoseEstimate();
        DistanceData currentData = new DistanceData(currentPose, distSensor.getDist(), currentAngle);
        dataStorage.add(currentData);
    }
}
