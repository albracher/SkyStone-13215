package org.firstinspires.ftc.teamcode;

import java.util.*;

public final class Point {
    public double x;    // x-coordinate
    public double y;    // y-coordinate

    // random point
    public Point() {
        x = 0;
        y = 0;
    }

    // point initialized from parameters
    public Point(double x1, double y1) {
        x = x1;
        y = y1;
    }

    // accessor methods
    public double x() { return x; }
    public double y() { return y; }
    public double r() { return Math.sqrt(x*x + y*y); }
    public double theta() { return Math.atan2(y, x); }

    // Euclidean distance between this point and that point
    public double distanceTo(Point that) {
        double dx = x - that.x;
        double dy = y - that.y;
        return Math.sqrt(dx*dx + dy*dy);
    }

    // return a string representation of this point
    public String toString() {
        return "(" + x + ", " + y + ")";
    }
}