package org.firstinspires.ftc.teamcode;

import

public class CartesianMovement {
    public static void goToPosition(double x, double y, double speed, double angle, double turnSpeed){
        double distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);

        double absoluteAngleToTarget = Math.atan2( y - worldYPosition, x - worldXPosition);

        double relativeAndleToPoint = angleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAndleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAndleToPoint) * distanceToTarget;
    }
}
