package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.hardware.Localizer;

import java.util.List;

@TeleOp(group="Test")
public class TestEncoder extends LinearOpMode {
    Localizer odo = new Localizer();
    Drivetrain drive = new Drivetrain();

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        odo.initialize(hardwareMap);
        drive.initialize(hardwareMap);

        ElapsedTime elapsedtime = new ElapsedTime();
        elapsedtime.reset();

        waitForStart();


        while (opModeIsActive()) {
            //if(gamepad1.x){ odo.resetIMU();}
            if(gamepad1.circle){ odo.resetEncoder(); }
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            //double botHeading = odo.heading();
            double botHeading = Localizer.theta;

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;


            drive.fl(frontLeftPower);
            drive.bl(backLeftPower);
            drive.fr(frontRightPower);
            drive.br(backRightPower);

            odo.positionArc();


            telemetry.addData("Status", "Running");

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

