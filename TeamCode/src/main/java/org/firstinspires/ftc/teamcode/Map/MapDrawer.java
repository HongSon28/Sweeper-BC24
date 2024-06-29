package org.firstinspires.ftc.teamcode.Map;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class MapDrawer {
    private FtcDashboard dashboard;
    public TelemetryPacket packet;
    public Canvas canvas;

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

    public void init() {
        // Clear and rotate field to make it Descartes-compatible
        canvas.clear();
        canvas.setRotation(-Math.PI / 2);

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

    private static void rotatePoints(double[] xPoints, double[] yPoints, double angle) {
        for (int i = 0; i < xPoints.length; i++) {
            double x = xPoints[i];
            double y = yPoints[i];
            xPoints[i] = x * Math.cos(angle) - y * Math.sin(angle);
            yPoints[i] = x * Math.sin(angle) + y * Math.cos(angle);
        }
    }

    public void drawBot(double x, double y, double rad) {
        double[] xPoints = {x * scale_factor, (x + bot_w) * scale_factor};
        double[] yPoints = {y * scale_factor, (y + bot_h) * scale_factor};
        rotatePoints(xPoints, yPoints, rad);
        canvas.setFill("green").fillPolygon(xPoints, yPoints);
    }

    public void drawPoint(double distance, double rad) {
        double x = distance * Math.sin(rad) * scale_factor;
        double y = distance * Math.cos(rad) * scale_factor;
        canvas.setFill("red").fillCircle(x, y, 1);
    }

    public void drawState(double x, double y, double rad, double dist) {
        //drawBot(x, y, rad);
        drawPoint(dist, rad);
        update();
    }
}
