package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Map.MapDrawer;

@Config
@TeleOp(name = "Test canvas")
public class TestCanvas extends LinearOpMode {
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
        FtcDashboard dashboard = FtcDashboard.getInstance();
        MapDrawer mapDrawer = new MapDrawer(dashboard, 8, 15);

        waitForStart();
        mapDrawer.init();

        //mapDrawer.drawBot(0, 0, Math.PI / 4);
        //mapDrawer.update();

        double x = 0;
        double y = 0;
        double yaw = Math.PI / 4;
        double distance = 10;
        while (opModeIsActive()) {
            for (int i = 0; i < 8; i++) {
                mapDrawer.drawState(x, y, yaw, distance);
                sleep(200);
                yaw += Math.PI / 4;
                distance += 2;
            }

            yaw = Math.PI / 4;
            distance = 20;
            x += 8;
            y -= 15;
            sleep(500);
        }
    }
}
