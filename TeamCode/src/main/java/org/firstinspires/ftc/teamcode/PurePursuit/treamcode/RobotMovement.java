package org.firstinspires.ftc.teamcode.PurePursuit.treamcode;

import static org.firstinspires.ftc.teamcode.PurePursuit.company.Robot.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.PurePursuit.company.Robot.worldXPosition;
import static org.firstinspires.ftc.teamcode.PurePursuit.company.Robot.worldYPosition;
import static org.firstinspires.ftc.teamcode.PurePursuit.treamcode.MathFunctions.AngleWrap;

import org.firstinspires.ftc.teamcode.PurePursuit.core.Point;

import java.util.ArrayList;

public class RobotMovement {

    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius){
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for (int i = 0; i < pathPoints.size() - 1; i++){
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get()
        }
    }


    public static void goToPosition(double x, double y, double movementSpeed){

        double distanceToTarget = Math.hypot(x-worldXPosition,y-worldYPosition);

        double absoluteAngleToTarget = Math.atan2(y-worldYPosition,x-worldXPosition);

        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        movement_x = movementXPower;
        movement_y = movementYPower;
    }
}
