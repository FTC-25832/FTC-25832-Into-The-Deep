package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.util.*;
import org.opencv.core.Point;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;


import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.util.expansion;
import org.firstinspires.ftc.teamcode.util.control;


@Autonomous(group="Auto")
public class Auto extends LinearOpMode {

    static Drivetrain drive = new Drivetrain();
    //Camera camera = new Camera();
    Localizer odo = new Localizer();
    UpperSlide upslide = new UpperSlide();

    @Override
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //camera.cameraStart(hardwareMap);
        upslide.initialize(hardwareMap);
        odo.initialize(hardwareMap);
        drive.initialize(hardwareMap);
        odo.resetEncoder();


        waitForStart();
        upslide.closeClaw();
        sleep(2000);
        //upslide.hang();

        while(opModeIsActive()){
            //startHang();
            park();
        }
    }

    public void startHang(){
        ArrayList<curvePoint> path = new ArrayList<>();
        path.add(new curvePoint(0, 0, 0.5, 1, 200, Math.toRadians(50), 1, 0));
        path.add(new curvePoint(0, 820, 0.5, 1, 200, Math.toRadians(50), 1, 0));
        upslide.pos1();
        curvePoint target;
        do{
            target = drive.followCurve(path);
            position();
        }while(target.moveSpeed!=0);

        upslide.pos2();
        do{
            upslide.updatePID();
        }while(Math.abs(upslide.pidController.lastError) > 200);
        upslide.openClaw();
    }

    public void push(){

    }

    public void park(){
        ArrayList<curvePoint> path = new ArrayList<>();
        path.add(new curvePoint(0, 0, 0.5, 0, 200, Math.toRadians(50), 1, 0));
        path.add(new curvePoint(800, 0, 0.5, 0, 200, Math.toRadians(50), 1, 0));
        curvePoint target;
        do{
            target = drive.followCurve(path);
            position();
        }while(target.moveSpeed!=0);

    }

    public void position(){
        Localizer.positionArc();
        telemetry.addData("X",Drivetrain.currX);
        telemetry.addData("Y",Drivetrain.currY);
        telemetry.addData("Heading",Drivetrain.currTheta);
        telemetry.update();
    }


}

