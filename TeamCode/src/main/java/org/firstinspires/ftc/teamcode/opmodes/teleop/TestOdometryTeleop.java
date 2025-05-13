package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.utils.hardware.Localizer;
import org.firstinspires.ftc.teamcode.utils.control.ControlHub;
import org.firstinspires.ftc.teamcode.utils.control.ExpansionHub;

import java.util.List;

@TeleOp(group="TeleOp")
public class TestOdometryTeleop extends LinearOpMode {
    Localizer odo = new Localizer();
    @Override
    public void runOpMode() throws InterruptedException {
            List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
            odo.initialize(hardwareMap, ControlHub.motor(0), ControlHub.motor(3), ControlHub.motor(2));

            ElapsedTime elapsedtime = new ElapsedTime();
            elapsedtime.reset();

            // Declare our motors
            // Make sure your ID's match your configuration
            DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, ExpansionHub.motor(2));
            DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, ExpansionHub.motor(3));
            DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, ExpansionHub.motor(1));
            DcMotor backRightMotor = hardwareMap.get(DcMotor.class, ExpansionHub.motor(0));

            // Reverse the right side motors. This may be wrong for your setup.
            backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            waitForStart();
            odo.resetEncoder();

            if (isStopRequested()) return;

            while (opModeIsActive()) {
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x; // Counteract imperfect strafing
                double rx = gamepad1.right_stick_x * 1.1;// * 1.1

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;


                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);

                if (gamepad1.circle) {
                    odo.resetEncoder();
                }

                Localizer.positionArc();

                telemetry.addData("Status", "Running");

                telemetry.addData("X", Localizer.X);
                telemetry.addData("Y", Localizer.Y);
                telemetry.addData("Heading", Localizer.angleWrap(Localizer.theta));

                telemetry.addData("Recording Rate (Hz)", 1 / elapsedtime.seconds());
                telemetry.addData("Loop Time (ms)", elapsedtime.milliseconds());

                elapsedtime.reset();

                //Telemetry
                telemetry.addData("Motors", "frontLeft (%.2f), frontRight (%.2f), backLeft (%.2f), backRight (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);

                displayGrid();

                telemetry.update();
            }
        }

    public void displayGrid() {
        final int SIZE = 11;
        final int SCALE = 60;

        char[][] grid = new char[SIZE][SIZE];


        for (int i = 0; i < SIZE; i++) {
            for (int j = 0; j < SIZE; j++) {
                grid[i][j] = '·';
            }
        }


        for (int i = 0; i < SIZE; i++) {
            grid[SIZE/2][i] = '-';
            grid[i][SIZE/2] = '|';
        }
        grid[SIZE/2][SIZE/2] = '+';


        int gridX = SIZE/2 + (int)Math.round(Localizer.X / SCALE);
        int gridY = SIZE/2 - (int)Math.round(Localizer.Y / SCALE);


        gridX = Math.min(Math.max(gridX, 0), SIZE-1);
        gridY = Math.min(Math.max(gridY, 0), SIZE-1);


        double heading = Localizer.angleWrap(Localizer.theta);
        char directionChar;


        if (heading >= -Math.PI/8 && heading < Math.PI/8) {
            directionChar = '→';
        } else if (heading >= Math.PI/8 && heading < 3*Math.PI/8) {
            directionChar = '↗';
        } else if (heading >= 3*Math.PI/8 && heading < 5*Math.PI/8) {
            directionChar = '↑';
        } else if (heading >= 5*Math.PI/8 && heading < 7*Math.PI/8) {
            directionChar = '↖';
        } else if (heading >= 7*Math.PI/8 || heading < -7*Math.PI/8) {
            directionChar = '←';
        } else if (heading >= -7*Math.PI/8 && heading < -5*Math.PI/8) {
            directionChar = '↙';
        } else if (heading >= -5*Math.PI/8 && heading < -3*Math.PI/8) {
            directionChar = '↓';
        } else {
            directionChar = '↘';
        }


        grid[gridY][gridX] = directionChar;

        StringBuilder display = new StringBuilder();

        display.append("    ");
        for (int i = 0; i < SIZE; i++) {
            int value = (i - SIZE/2) * SCALE;
            display.append(String.format("%3d ", value));
        }
        display.append("\n");


        for (int i = 0; i < SIZE; i++) {
            int value = -(i - SIZE/2) * SCALE;
            display.append(String.format("%3d ", value));

            for (int j = 0; j < SIZE; j++) {
                display.append(" " + grid[i][j] + " ");
            }
            display.append("\n");
        }

        telemetry.addLine(display.toString());
    }
}

