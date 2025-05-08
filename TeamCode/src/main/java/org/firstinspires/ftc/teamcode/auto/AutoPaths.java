package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class AutoPaths {
        // Store the starting poses for different paths
        public static final Pose2d START_POSE = new Pose2d(0, 62.5, -Math.PI / 2);

        // Helper method to reset robot pose
        public static void resetPose(MecanumDrive drive) {
                drive.localizer.setPose(START_POSE);
        }

        public static Action getHangFirstPath(MecanumDrive drive) {
                return drive.actionBuilder(START_POSE)
                                .splineToConstantHeading(new Vector2d(0, 30), -Math.PI / 2)
                                .build();
        }

        // First block (x = -40.5)
        public static Action getGotoPreplaced(MecanumDrive drive) {
                return drive.actionBuilder(new Pose2d(0, 30, -Math.PI / 2))
                                .strafeToSplineHeading(new Vector2d(-40.5, 40.5), Math.toRadians(-135))
                                .build();
        }

        public static Action getRotateToTeamBox(MecanumDrive drive) {
                return drive.actionBuilder(new Pose2d(-40.5, 40.5, Math.toRadians(-135)))
                                .turn(Math.toRadians(-120))
                                .build();
        }

        // Second block (x = -52)
        public static Action getGotoPreplacedSecond(MecanumDrive drive) {
                return drive.actionBuilder(new Pose2d(-40.5, 40.5, Math.toRadians(105)))
                                .splineTo(new Vector2d(-52, 40.5), Math.toRadians(-135))
                                .build();
        }

        public static Action getRotateToTeamBoxSecond(MecanumDrive drive) {
                return drive.actionBuilder(new Pose2d(-52, 40.5, Math.toRadians(-135)))
                                .turn(Math.toRadians(-120))
                                .build();
        }

        // Third block (x = -61)
        public static Action getGotoPreplacedThird(MecanumDrive drive) {
                return drive.actionBuilder(new Pose2d(-52, 40.5, Math.toRadians(105)))
                                .splineTo(new Vector2d(-61, 40.5), Math.toRadians(-135))
                                .build();
        }

        public static Action getRotateToTeamBoxThird(MecanumDrive drive) {
                return drive.actionBuilder(new Pose2d(-61, 40.5, Math.toRadians(-135)))
                                .turn(Math.toRadians(-120))
                                .build();
        }

        public static Action getStart(MecanumDrive drive) {
                return drive.actionBuilder(new Pose2d(-61, 40.5, Math.toRadians(105)))
                                .strafeToSplineHeading(new Vector2d(-47, 62.5), Math.toRadians(-90))
                                .build();
        }

        public static Action getGoToSpecimenHang(MecanumDrive drive) {
                return drive.actionBuilder(new Pose2d(-47, 62.5, Math.toRadians(-90)))
                                .splineToConstantHeading(new Vector2d(-10, 30), -Math.PI / 2)
                                .build();
        }

        public static Action getGoToTeamBox(MecanumDrive drive) {
                return drive.actionBuilder(new Pose2d(-10, 30, Math.toRadians(-90)))
                                .strafeToConstantHeading(new Vector2d(-47, 62.5))
                                .build();
        }

        // Legacy paths (can be removed if not needed)
        public static Action getOriginalTestPath(MecanumDrive drive) {
                return drive.actionBuilder(START_POSE)
                                .splineTo(new Vector2d(30, 30), Math.PI / 2)
                                .splineTo(new Vector2d(0, 60), Math.PI)
                                .build();
        }

        public static Action getSquarePath(MecanumDrive drive) {
                return drive.actionBuilder(START_POSE)
                                .splineTo(new Vector2d(30, 0), 0)
                                .splineTo(new Vector2d(30, 30), Math.PI / 2)
                                .splineTo(new Vector2d(0, 30), Math.PI)
                                .splineTo(new Vector2d(0, 0), -Math.PI / 2)
                                .build();
        }

        public static Action getChicanePath(MecanumDrive drive) {
                return drive.actionBuilder(START_POSE)
                                .splineTo(new Vector2d(20, 20), Math.PI / 4)
                                .splineTo(new Vector2d(40, 0), -Math.PI / 4)
                                .splineTo(new Vector2d(60, 20), Math.PI / 4)
                                .build();
        }

        public static Action getPrePlaced(MecanumDrive drive) {
                return drive.actionBuilder(START_POSE)
                                .splineTo(new Vector2d(-51, 30), Math.PI / 2)
                                .splineTo(new Vector2d(-51, 56), Math.PI / 2)
                                .splineTo(new Vector2d(-60, 30), Math.PI / 2)
                                .splineTo(new Vector2d(-60, 56), Math.PI / 2)
                                .splineTo(new Vector2d(-65, 30), Math.PI / 2)
                                .splineTo(new Vector2d(-65, 56), Math.PI / 2)
                                .splineTo(new Vector2d(0, 30), Math.PI / 2)
                                .splineTo(new Vector2d(-49, 56), Math.PI / 2)
                                .splineTo(new Vector2d(0, 30), Math.PI / 2)
                                .splineTo(new Vector2d(-49, 56), Math.PI / 2)
                                .splineTo(new Vector2d(0, 30), Math.PI / 2)
                                .splineTo(new Vector2d(-49, 56), Math.PI / 2)
                                .splineTo(new Vector2d(0, 30), Math.PI / 2)
                                .splineTo(new Vector2d(-49, 56), Math.PI / 2)
                                .build();
        }
}
