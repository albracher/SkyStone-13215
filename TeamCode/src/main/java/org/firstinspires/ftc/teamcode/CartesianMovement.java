package org.firstinspires.ftc.teamcode;

public class CartesianMovement {
    public static void goToPosition(double x, double y, double speed, double angle, double turnSpeed){
        double distanceToTarget = Math.hypot(x - ExperimentalAutonMap.robotXPos, y - ExperimentalAutonMap.robotYPos);

        double absoluteAngleToTarget = Math.atan2( y - ExperimentalAutonMap.robotYPos, x - ExperimentalAutonMap.robotXPos);

        double relativeAngleToPoint = PurePursuitMath.angleWrap(absoluteAngleToTarget - (ExperimentalAutonMap.heading - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        ExperimentalAutonMap.xMovement(movementXPower * speed);
        ExperimentalAutonMap.yMovement(movementYPower * speed);
    }
}
