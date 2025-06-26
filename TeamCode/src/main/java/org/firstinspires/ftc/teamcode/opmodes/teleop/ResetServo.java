package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This OpMode resets the PodM gimbal servo to its default position.
 * Press A button to reset the servo to position 0.5 (center position).
 * Press B button to reset the servo to position 0.0 (minimum position).
 * Press Y button to reset the servo to position 1.0 (maximum position).
 */
@TeleOp(group="TeleOp")
public class ResetServo extends LinearOpMode {
    // Declare OpMode members
    private ServoImplEx podMServo = null;
    private ElapsedTime runtime = new ElapsedTime();

    // Constants for servo positions
    private static final double CENTER_POSITION = 0.5;   // Center position (default reset)
    private static final double MIN_POSITION = 0;      // Minimum position
    private static final double MAX_POSITION = 1.0;      // Maximum position
    PwmControl.PwmRange range = new PwmControl.PwmRange(500, 2500);
    @Override
    public void runOpMode() {
        podMServo = hardwareMap.get(ServoImplEx.class, "PodM");
        podMServo.setPwmRange(range);
        // Wait for the user to press the start button
        waitForStart();
        runtime.reset();

        // Default to center position on start
        if (podMServo != null) {
            podMServo.setPosition(CENTER_POSITION);
        }

        // Run until the end of the match
        while (opModeIsActive()) {
            // Check for button presses to reset servo
            if (gamepad1.a) {
                if (podMServo != null) {
                    podMServo.setPosition(CENTER_POSITION);
                    telemetry.addData("Action", "Reset to center position (0.5)");
                    sleep(500); // Debounce delay
                }
            }

            if (gamepad1.b) {
                if (podMServo != null) {
                    podMServo.setPosition(MIN_POSITION);
                    telemetry.addData("Action", "Reset to minimum position (0.0)");
                    sleep(500); // Debounce delay
                }
            }

            if (gamepad1.y) {
                if (podMServo != null) {
                    podMServo.setPosition(MAX_POSITION);
                    telemetry.addData("Action", "Reset to maximum position (1.0)");
                    sleep(500); // Debounce delay
                }
            }

            if (gamepad1.x) {
                telemetry.addData("Status", "Exiting...");
                telemetry.update();
                sleep(1000);
                break;
            }

            // Display current servo position and runtime
            if (podMServo != null) {
                telemetry.addData("Servo Position", "%.2f", podMServo.getPosition());
            } else {
                telemetry.addData("Servo Status", "Not initialized");
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("Runtime", "%.2f seconds", runtime.seconds());
            telemetry.addData("Instructions", "Press A: center position");
            telemetry.addData("Instructions", "Press B: minimum position");
            telemetry.addData("Instructions", "Press Y: maximum position");
            telemetry.update();
        }
    }
}