package org.firstinspires.ftc.teamcode.Map;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.Constants.MAP.*;

public class MapDrawer {
    private LinearOpMode opMode;
    private FtcDashboard dashboard;
    public TelemetryPacket packet;
    public Canvas canvas;

    private Logger logger = new Logger(true);

    double bot_w, bot_h, scale_factor;

    public MapDrawer(FtcDashboard _dashboard, double bot_width, double bot_height) {
        // Set dashboard object
        this.dashboard = _dashboard;

        // Create telemetry packet
        packet = new TelemetryPacket(false);

        // Set canvas object
        canvas = packet.fieldOverlay();

        // Set bot width and height
        bot_w = bot_width;
        bot_h = bot_height;
    }

    public MapDrawer(FtcDashboard _dashboard, double bot_width, double bot_height, double _scale_factor) {
        // Set dashboard object
        this.dashboard = _dashboard;

        // Create telemetry packet
        packet = new TelemetryPacket(false);

        // Set canvas object
        canvas = packet.fieldOverlay();

        // Set bot width and height
        bot_w = bot_width;
        bot_h = bot_height;
        scale_factor = _scale_factor;
    }

    public void init(LinearOpMode _opMode) {
        // Set opMode object
        opMode = _opMode;

        // Clear field
        canvas.clear();

        // Draw field outer frame
        canvas.setFill("white").fillRect(-72, -72, 144, 144);

        // Draw bot at initial position of 0,0
        //drawBot(0, 0, 0);

        // Send packet
        update();
    }

    public void update() {
        dashboard.sendTelemetryPacket(packet);
    }

    public double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
    }

    private static void rotatePoints(double[] xPoints, double[] yPoints, double angle) {
        for (int i = 0; i < xPoints.length; i++) {
            double x = xPoints[i];
            double y = yPoints[i];
            xPoints[i] = x * Math.cos(angle) - y * Math.sin(angle);
            yPoints[i] = x * Math.sin(angle) + y * Math.cos(angle);
        }
    }

    public void drawBot(double x, double y, double rad) {
        /*double[] xPoints = {x * scale_factor, (x + bot_w) * scale_factor};
        double[] yPoints = {y * scale_factor, (y + bot_h) * scale_factor};
        rotatePoints(xPoints, yPoints, rad);
        canvas.setFill("green").fillPolygon(xPoints, yPoints);*/
        canvas.setFill("green").strokeLine(x, y, x + Math.cos(rad) * 5, y + Math.sin(rad) * 5);
    }

    public void drawPoint(double x, double y, double distance, double rad) {
        double d = distance + BOT_CENTER_TO_DISTANCE_SENSOR;
        double p_x = d * Math.cos(rad) * scale_factor;
        double p_y = d * Math.sin(rad) * scale_factor;
        canvas.setFill("red").fillCircle(x + p_x, y + p_y, 1);
        logger.logO(x + p_x, y + p_y);
        //opMode.telemetry.addData("point", "%f, %f", x + p_x, y + p_y);
        //opMode.telemetry.update();
    }

    public void drawState(double x, double y, double rad, double dist) {
        if (dist <= 0) return;

        logger.logI(x, y, rad, dist);
        //double ang = angleWrap(rad + Math.PI/2);
        //drawBot(x, y, rad);
        drawPoint(x, y, dist, rad);
        update();
    }
}
