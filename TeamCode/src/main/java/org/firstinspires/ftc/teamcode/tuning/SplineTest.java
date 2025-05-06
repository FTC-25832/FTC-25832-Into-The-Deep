package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.auto.AutoPaths;

public final class SplineTest extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
                MecanumDrive drive = new MecanumDrive(hardwareMap, AutoPaths.START_POSE);

                waitForStart();

                // Run original test path (semicircle)
                Actions.runBlocking(AutoPaths.getOriginalTestPath(drive));

                // Reset pose for square path test
                AutoPaths.resetPose(drive);

                // Run square path test
                Actions.runBlocking(AutoPaths.getSquarePath(drive));

                // Reset pose for chicane path test
                AutoPaths.resetPose(drive);

                // Run chicane path test
                Actions.runBlocking(AutoPaths.getChicanePath(drive));
        }
}
