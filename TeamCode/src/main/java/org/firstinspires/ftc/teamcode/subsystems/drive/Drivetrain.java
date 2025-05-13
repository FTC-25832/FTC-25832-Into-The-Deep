package org.firstinspires.ftc.teamcode.subsystems.drive;

import android.service.controls.Control;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;
import org.firstinspires.ftc.teamcode.utils.control.ControlHub;
import org.firstinspires.ftc.teamcode.utils.hardware.Localizer;
import org.firstinspires.ftc.teamcode.utils.math.curvePoint;
import org.firstinspires.ftc.teamcode.utils.control.ExpansionHub;
import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase;
import org.opencv.core.Point;

import java.util.ArrayList;

public class Drivetrain extends SubsystemBase {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private static double currX, currY, currTheta;

    public Drivetrain() {
        super("drivetrain");
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotor.class, ControlHub.motor(2));
        backLeftMotor = hardwareMap.get(DcMotor.class, ControlHub.motor(3));
        frontRightMotor = hardwareMap.get(DcMotor.class, ExpansionHub.motor(1));
        backRightMotor = hardwareMap.get(DcMotor.class, ControlHub.motor(0));

        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodic(TelemetryPacket packet) {
        // Add motor powers to telemetry
        packet.put("drivetrain/frontLeftPower", frontLeftMotor.getPower());
        packet.put("drivetrain/frontRightPower", frontRightMotor.getPower());
        packet.put("drivetrain/backLeftPower", backLeftMotor.getPower());
        packet.put("drivetrain/backRightPower", backRightMotor.getPower());

        // Add position data
        packet.put("drivetrain/x", currX);
        packet.put("drivetrain/y", currY);
        packet.put("drivetrain/theta", currTheta);
    }

    /**
     * Set the power of the front left motor
     */
    public void setFrontLeft(double power) {
        frontLeftMotor.setPower(power);
    }

    /**
     * Set the power of the front right motor
     */
    public void setFrontRight(double power) {
        frontRightMotor.setPower(power);
    }

    /**
     * Set the power of the back left motor
     */
    public void setBackLeft(double power) {
        backLeftMotor.setPower(power);
    }

    /**
     * Set the power of the back right motor
     */
    public void setBackRight(double power) {
        backRightMotor.setPower(power);
    }

    /**
     * Set the power of all drive motors
     */
    public void setMotorPowers(double fl, double fr, double bl, double br) {
        setFrontLeft(fl);
        setFrontRight(fr);
        setBackLeft(bl);
        setBackRight(br);
    }

    /**
     * Drive using field-centric controls
     */
    public void drive(double x, double y, double rx, double heading) {
        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        rotX = rotX * 1.1; // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when at least
        // one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double frontLeftPower = (rotY + rotX + rx) / denominator
                * ConfigVariables.General.DRIVETRAIN_SPEED_MULTIPLIERFORLIMIT;
        double backLeftPower = (rotY - rotX + rx) / denominator
                * ConfigVariables.General.DRIVETRAIN_SPEED_MULTIPLIERFORLIMIT;
        double frontRightPower = (rotY - rotX - rx) / denominator
                * ConfigVariables.General.DRIVETRAIN_SPEED_MULTIPLIERFORLIMIT;
        double backRightPower = (rotY + rotX - rx) / denominator
                * ConfigVariables.General.DRIVETRAIN_SPEED_MULTIPLIERFORLIMIT;

        setMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    /**
     * Move to a target position using path following
     */
    public void moveToPosition(double x, double y, double angle, double moveSpeed, double turnSpeed) {
        Localizer.positionArc();

        currX = -Localizer.Y;
        currY = Localizer.X;
        currTheta = Localizer.theta + Math.toRadians(90);

        double distance = Math.hypot(x - currX, y - currY);
        double absoluteAngle = Math.atan2(y - currY, x - currX);
        double relAngle = Localizer.angleWrap(absoluteAngle - currTheta + Math.toRadians(90));

        double relX = Math.cos(relAngle) * distance;
        double relY = Math.sin(relAngle) * distance;

        double powerX = (relX / (Math.abs(relX) + Math.abs(relY))) * moveSpeed;
        double powerY = (relY / (Math.abs(relX) + Math.abs(relY))) * moveSpeed;

        double relTurn = relAngle - Math.toRadians(180) + angle;
        double rx = Range.clip(relTurn / Math.toRadians(30), -1, 1) * turnSpeed;

        setMotorPowers(
                powerY + powerX - rx,
                powerY - powerX + rx,
                powerY - powerX - rx,
                powerY + powerX + rx);
    }

    /**
     * Follow a curve defined by control points
     */
    public curvePoint followCurve(ArrayList<curvePoint> allPoints) {
        curvePoint followPoint = followPointPath(allPoints, new Point(currX, currY), allPoints.get(0).followDistance);

        double targetAngle = followPoint.end != 0 ? followPoint.end : Math.toRadians(90);
        moveToPosition(followPoint.x, followPoint.y, targetAngle, followPoint.moveSpeed, followPoint.turnSpeed);

        return followPoint;
    }

    private curvePoint followPointPath(ArrayList<curvePoint> path, Point robot, double followRadius) {
        curvePoint followMe = new curvePoint(path.get(0));
        double endAngle = path.get(path.size() - 1).end;

        // Add lookahead point
        curvePoint p1 = path.get(path.size() - 1);
        curvePoint p2 = path.get(path.size() - 2);
        double m = (p2.y - p1.y) / (p2.x - p1.x);
        double b = p1.y - m * p1.x;
        double r = 20;

        // Calculate intersection points
        double xRoot1 = (-m * b + Math.sqrt(Math.pow(r, 2) + Math.pow(r, 2) * Math.pow(m, 2) - Math.pow(b, 2)))
                / (Math.pow(m, 2) + 1);
        double xRoot2 = (-m * b - Math.sqrt(Math.pow(r, 2) + Math.pow(r, 2) * Math.pow(m, 2) - Math.pow(b, 2)))
                / (Math.pow(m, 2) + 1);

        xRoot1 += p1.x;
        xRoot2 += p1.x;

        double yRoot1 = (xRoot1 * m + b);
        double yRoot2 = (xRoot2 * m + b);

        // Choose the better intersection point
        Point targetPoint;
        if (length(xRoot1, yRoot1, p2.x, p2.y) < length(xRoot2, yRoot2, p2.x, p2.y)) {
            path.add(new curvePoint(xRoot2, yRoot2, 1, 1, 50, Math.toRadians(50), 1, endAngle));
            targetPoint = new Point(xRoot2, yRoot2);
        } else {
            path.add(new curvePoint(xRoot1, yRoot1, 1, 1, 50, Math.toRadians(50), 1, endAngle));
            targetPoint = new Point(xRoot1, yRoot1);
        }

        // Find closest intersection point
        ArrayList<Point> intersections = Localizer.intersect(robot, followRadius, new Point(p1.x, p1.y), targetPoint);
        double closest = Double.MAX_VALUE;
        for (Point intersection : intersections) {
            double distance = Math
                    .sqrt(Math.pow(intersection.x - targetPoint.x, 2) + Math.pow(intersection.y - targetPoint.y, 2));
            if (distance < closest) {
                closest = distance;
                followMe.setPoint(intersection);
            }
        }

        path.remove(path.size() - 1);

        // Check if we're near the end point
        if (within(robot, new Point(path.get(path.size() - 1).x, path.get(path.size() - 1).y), 100)) {
            followMe.moveSpeed = 0;
            followMe.end = endAngle;
        }

        return followMe;
    }

    private double length(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }

    private boolean within(Point p, Point mid, double r) {
        double y1 = Math.sqrt(Math.pow(r, 2) - Math.pow(p.x - mid.x, 2));
        double y2 = -Math.sqrt(Math.pow(r, 2) - Math.pow(p.x - mid.x, 2));
        y1 += mid.y;
        y2 += mid.y;
        return (p.y <= y1 && p.y >= y2);
    }

    @Override
    public void stop() {
        // Stop all drive motors
        setMotorPowers(0, 0, 0, 0);

        // Set motor modes to help prevent coasting
        // frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
