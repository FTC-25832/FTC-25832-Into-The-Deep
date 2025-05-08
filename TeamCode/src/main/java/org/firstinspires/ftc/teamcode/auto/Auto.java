package org.firstinspires.ftc.teamcode.auto.classes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.outtake.CloseClaw;
import org.firstinspires.ftc.teamcode.commands.outtake.OpenClaw;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

import java.util.function.BooleanSupplier;

@Autonomous(preselectTeleOp = "SoloKota")
public class Auto extends OpMode {
    private AutoSelection autoSelection;
    private BaseAuto auto;

    private MultipleTelemetry telemetryA;

    @Override
    public void init() {
        RobotHardware.reset(hardwareMap);
        RobotHardware.getInstance().servoInit();
        RobotHardware.getInstance().setLightColor(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        autoSelection = new AutoSelection(gamepad1);

        telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void init_loop() {
        autoSelection.updateTelemetry(telemetryA);
        telemetryA.update();

        if (gamepad1.a) {
            CommandScheduler.getInstance().schedule(new CloseClaw());
        }
        if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.b) {
            CommandScheduler.getInstance().schedule(new OpenClaw());
        }
        CommandScheduler.getInstance().run();
    }

    @Override
    public void start() {
        RobotHardware.getInstance().startPids();
        autoSelection.interrupt();

        switch (autoSelection.getAuto()) {
            case fullSample:
                auto = new Sample(autoSelection.getColor());
                break;
            case specimen:
                auto = new Specimen(autoSelection.getColor());
                break;
        }
    }

    @Override
    public void loop() {
        OuttakeSubsystem.getInstance().updatePID();
        IntakeSubsystem.getInstance().updatePID();
        auto.updateTelemetry(telemetryA);
        OuttakeSubsystem.getInstance().updateTelemetry(telemetryA);
        telemetryA.update();
    }

    @Override
    public void stop() {
        RobotHardware.getInstance().interrupt();
        if (auto != null) {
            auto.interrupt();
        }
    }
}