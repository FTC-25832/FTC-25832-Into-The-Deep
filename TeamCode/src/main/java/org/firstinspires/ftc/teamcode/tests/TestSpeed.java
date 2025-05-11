package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.hardware.Localizer;

import java.util.List;

@TeleOp(group="Test")

public class TestSpeed extends LinearOpMode {
    Localizer odo = new Localizer();

    @Override
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        odo.initialize(hardwareMap);

        ElapsedTime elapsedtime = new ElapsedTime();
        elapsedtime.reset();

        waitForStart();
        while (opModeIsActive()) {
            Localizer.positionArc();
            telemetry.addData("X",Localizer.X);
            telemetry.addData("Y",Localizer.Y);
            telemetry.addData("Heading",Localizer.angleWrap(Localizer.theta));


            telemetry.addData("Hertz", 1/elapsedtime.seconds());
            telemetry.addData("Loop Time", elapsedtime.seconds());

            telemetry.update();
            elapsedtime.reset();
        }
    }
}
