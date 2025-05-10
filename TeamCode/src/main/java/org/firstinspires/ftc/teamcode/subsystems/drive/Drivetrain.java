package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.hardware.Localizer;
import org.firstinspires.ftc.teamcode.utils.math.curvePoint;
import org.firstinspires.ftc.teamcode.utils.control.ExpansionHub;
import org.opencv.core.Point;
import java.util.ArrayList;


public class Drivetrain{
    DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    HardwareMap hardwareMap;

    public static double currX, currY, currTheta;
    public void initialize(HardwareMap map){
        hardwareMap = map;
        frontLeftMotor = hardwareMap.get(DcMotor.class, ExpansionHub.motor(2));
        backLeftMotor = hardwareMap.get(DcMotor.class, ExpansionHub.motor(3));
        frontRightMotor = hardwareMap.get(DcMotor.class, ExpansionHub.motor(1));
        backRightMotor = hardwareMap.get(DcMotor.class, ExpansionHub.motor(0));
        
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        */

    }

    //public double heading(){ return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); }
    public void fl(double power){ frontLeftMotor.setPower(power); }
    public void fr(double power){ frontRightMotor.setPower(power); }
    public void bl(double power){ backLeftMotor.setPower(power); }
    public void br(double power){ backRightMotor.setPower(power); }


    public curvePoint followCurve(ArrayList<curvePoint> all){
        curvePoint follow = followPointPath(all, new Point(currX,currY), all.get(0).followDistance);
        if(follow.end!=0) {move(follow.x, follow.y, follow.end, follow.moveSpeed, follow.turnSpeed);}
        else {move(follow.x, follow.y, Math.toRadians(90), follow.moveSpeed, follow.turnSpeed);}
        return follow;
    }

    public curvePoint followPointPath(ArrayList<curvePoint> path, Point robot, double followR){
        curvePoint follow = new curvePoint(path.get(0));
        double endAngle = path.get(path.size()-1).end;

        curvePoint p1 = path.get(path.size()-1);
        curvePoint p2 = path.get(path.size()-2);
        double m = (p2.y - p1.y)/(p2.x - p1.x); // potential 0 division error
        double b = p1.y - m*p1.x;
        double r = 20;
        // solve y=mx+b and x^2+y^2=r^2
        double xRoot1 = (-m*b + Math.sqrt(Math.pow(r,2) + Math.pow(r,2)*Math.pow(m,2) - Math.pow(b,2)))/(Math.pow(m,2) + 1);
        double xRoot2 = (-m*b - Math.sqrt(Math.pow(r,2) + Math.pow(r,2)*Math.pow(m,2) - Math.pow(b,2)))/(Math.pow(m,2) + 1);

        xRoot1 += p1.x;
        xRoot2 += p1.x;

        double yRoot1 = (xRoot1*m+b);
        double yRoot2 = (xRoot2*m+b);

        if(length(xRoot1, yRoot1, p2.x, p2.y) < length(xRoot2, yRoot2, p2.x, p2.y)){ path.add(new curvePoint(xRoot2, yRoot2, 1, 1, 50, Math.toRadians(50), 1, endAngle));}
        else{ path.add(new curvePoint(xRoot1, yRoot1, 1, 1, 50, Math.toRadians(50), 1, endAngle)); }


        for(int i=0; i<path.size()-1;i++){
            curvePoint start = path.get(i);
            curvePoint end = path.get(i+1);

            ArrayList<Point> intersection = Localizer.intersect(robot, followR, start.toPoint(), end.toPoint());
            double closest = Double.MAX_VALUE;
            for(Point currIntersect : intersection){
                double distance = Math.sqrt(Math.pow(currIntersect.x-end.x,2)+Math.pow(currIntersect.y-end.y,2));
                if(distance < closest){
                    closest = distance;
                    follow.setPoint(currIntersect);
                }
            }
        }
        
        path.remove(path.size()-1);
        if(within(robot, new Point(path.get(path.size()-1).x, path.get(path.size()-1).y),100)){
            follow.moveSpeed = 0;
            follow.end = endAngle;
        }
        return follow;
    }
    public void move(double x, double y, double angle, double moveSpeed, double turnSpeed){
        Localizer.positionArc();

        currX = -Localizer.Y;
        currY = Localizer.X;
        currTheta = Localizer.theta+Math.toRadians(90);

        double distance = Math.hypot(x-currX,y-currY);
        double absoluteAngle = Math.atan2(y-currY,x-currX);
        double relAngle = Localizer.angleWrap(absoluteAngle - currTheta + Math.toRadians(90));

        double relX = Math.cos(relAngle)*distance;
        double relY = Math.sin(relAngle)*distance;

        double powerX = (relX / (Math.abs(relX) + Math.abs(relY))) * moveSpeed;
        double powerY = (relY / (Math.abs(relX) + Math.abs(relY))) * moveSpeed;

        double relTurn = relAngle - Math.toRadians(180) + angle;

        double rx = Range.clip(relTurn/Math.toRadians(30),-1,1) * turnSpeed;

        double frontLeftPower = (powerY + powerX - rx);
        double backLeftPower = (powerY - powerX - rx);
        double frontRightPower = (powerY - powerX + rx);
        double backRightPower = (powerY + powerX + rx);


        fl(frontLeftPower);
        bl(backLeftPower);
        fr(frontRightPower);
        br(backRightPower);

    }


    public double length(double x1, double y1, double x2, double y2){
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }

    public boolean within(Point p, Point mid, double r){
        double y1 = Math.sqrt(Math.pow(r, 2) - Math.pow(p.x - mid.x,2));
        double y2 = -Math.sqrt(Math.pow(r, 2) - Math.pow(p.x - mid.x,2));
        y1 += mid.y;
        y2 += mid.y;
        return (p.y<=y1 && p.y>=y2);
    }

    /*
    public double PID(double reference, double state) {
        double error = reference - state;
        //telemetry.addData("Error: ", error);
        //integralSum += error * timer.seconds();
        //double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        //timer.reset();
        //double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
    */
    
}
