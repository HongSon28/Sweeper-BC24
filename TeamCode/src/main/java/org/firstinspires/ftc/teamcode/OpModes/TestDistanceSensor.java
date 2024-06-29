package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DistSensor;

@TeleOp(name = "Test distance sensor")
public class TestDistanceSensor extends LinearOpMode {
    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     *
     * @throws InterruptedException When the OpMode is stopped while calling a method
     *                              that can throw {@link InterruptedException}
     */
    @Override
    public void runOpMode() throws InterruptedException {
        DistSensor dist = new DistSensor(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Raw distance data", dist.getRawDist());
            telemetry.addData("Processed distance data", dist.getDist());
            telemetry.update();
        }
    }
}
