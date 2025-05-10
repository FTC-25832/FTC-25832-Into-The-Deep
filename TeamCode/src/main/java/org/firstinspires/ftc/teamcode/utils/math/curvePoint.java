package org.firstinspires.ftc.teamcode.utils.math;

import org.opencv.core.Point;


public class curvePoint {
    public double x;
    public double y;
    public double moveSpeed;
    public double turnSpeed;
    public double followDistance;
    public double slowRadians;
    public double slowAmount;
    public double end;

    public curvePoint(double x, double y,
                      double moveSpeed,
                      double turnSpeed,
                      double followDistance,
                      double slowRadians,
                      double slowAmount,
                      double end) {
        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        this.slowRadians = slowRadians;
        this.slowAmount = slowAmount;
        this.end = end;
    }

    public curvePoint(curvePoint point) {
        x = point.x;
        y = point.y;
        moveSpeed = point.moveSpeed;
        turnSpeed = point.turnSpeed;
        followDistance = point.followDistance;
        slowRadians = point.slowRadians;
        slowAmount = point.slowAmount;
        end = point.end;
    }

    public Point toPoint(){
        return new Point(x ,y);
    }

    public void setPoint(Point point){
        x = point.x;
        y = point.y;
    }
}
