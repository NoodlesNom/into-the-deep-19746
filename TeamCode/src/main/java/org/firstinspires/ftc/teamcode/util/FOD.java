package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FOD {

    private double theta = 0;
    private double r = 0;
    private double x = 0;
    private double y = 0;
    private double new_x = 0;
    private double new_y = 0;
    private double adjustAngle = 0;
    private double heading = 0;

    public FOD(double adjust) {
        adjustAngle = Math.toRadians(adjust);
    }

    public FOD() {
        this(0.0);
    }

    public void setAdjust(double adjust) {
        adjustAngle = Math.toRadians(adjust);
        convert();
    }

    public void setCar(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = Math.toRadians(heading);
        convert();
    }

    private void convert() {
        r = Math.sqrt((x * x) + (y * y));
        theta = Math.atan2(y, x);
        theta = AngleUnit.RADIANS.normalize(theta - AngleUnit.RADIANS.normalize(heading - adjustAngle));
        new_x = r * Math.cos(theta);
        new_y = r * Math.sin(theta);
    }

    public double getNewX() {
        return new_x;
    }

    public double getNewY() {
        return new_y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return Math.toDegrees(theta);
    }

    public double getR() {
        return r;
    }

    public double getHeading() {
        return Math.toDegrees(heading);
    }

    public double getAdjust() {
        return Math.toDegrees(adjustAngle);
    }
}
