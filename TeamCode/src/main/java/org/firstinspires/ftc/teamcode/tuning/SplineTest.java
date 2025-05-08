package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.auto.AutoPaths;

public final class SplineTest extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
                // MecanumDrive drive = new MecanumDrive(hardwareMap, AutoPaths.START_POSE);
                MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-47, 62.5, Math.toRadians(-90)));

                waitForStart();

                // Run the full auto sequence
                Actions.runBlocking(new SequentialAction(
                                // AutoPaths.getHangFirstPath(drive),
                                // AutoPaths.getGotoPreplaced(drive),
                                // AutoPaths.getRotateToTeamBox(drive),

                                // // Second block sequence
                                // AutoPaths.getGotoPreplacedSecond(drive),
                                // AutoPaths.getRotateToTeamBoxSecond(drive),

                                // // Third block sequence
                                // AutoPaths.getGotoPreplacedThird(drive),
                                // AutoPaths.getRotateToTeamBoxThird(drive),

                                // // Specimen handling sequence
                                // AutoPaths.getStart(drive),
                                // AutoPaths.getGoToSpecimenHang(drive),
                                // AutoPaths.getGoToTeamBox(drive),

                                // Repeat specimen handling
                                AutoPaths.getGoToSpecimenHang(drive),
                                AutoPaths.getGoToTeamBox(drive),
                                AutoPaths.getGoToSpecimenHang(drive),
                                AutoPaths.getGoToTeamBox(drive),
                                AutoPaths.getGoToSpecimenHang(drive),
                                AutoPaths.getGoToTeamBox(drive)));

                // Legacy test paths (commented out)
                // Actions.runBlocking(AutoPaths.getOriginalTestPath(drive));
                // AutoPaths.resetPose(drive);
                // Actions.runBlocking(AutoPaths.getSquarePath(drive));
                // AutoPaths.resetPose(drive);
                // Actions.runBlocking(AutoPaths.getChicanePath(drive));
        }
}
