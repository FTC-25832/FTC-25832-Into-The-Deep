package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class AutoPaths {
        // Configuration values
        public static double testYValue = 61.5;
        public static double testYValue2 = 33;
        public static double testYValue3 = 61.5;
        public static double testYValue4 = 33;
        public static double thirdSpecimenOffset = 3.5;
        public static double fourthSpecimenOffset = 3.5;
        public static double clipOffset = 3.5;
        public static double testXValue = -45;
        public static int clipDelay = 200;
        public static int extendLength = 515;
        public static double neutralPitch = 0.15;
        public static double neutralYaw = 1;
        public static int grabDelay = 100;
        public static int pickUpDelay = 200;
        public static int dropOffDelay = 200;
        public static double extendDelay = 1;
        public static double botLength = 15.748;
        public static double botWidth = 13.386;
        public static double TRACK_WIDTH = 11.25286365;

        // Store the starting poses for different paths
        public static final Pose2d START_POSE = new Pose2d(-17.2, 63, Math.toRadians(-90));

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

        // Primary auto path with delays for robot actions
        public static Action getPrimaryAutoPath(MecanumDrive drive) {
                return drive.actionBuilder(START_POSE)
                                .strafeToConstantHeading(new Vector2d(-8, testYValue2))
                                .waitSeconds(clipDelay / 1000.0)
                                .strafeToLinearHeading(new Vector2d(-39, 36), Math.toRadians(0))
                                .strafeTo(new Vector2d(-39, 21))
                                .waitSeconds(pickUpDelay / 1000.0)
                                .strafeToLinearHeading(new Vector2d(-41, 45), Math.toRadians(310))
                                .strafeToLinearHeading(new Vector2d(-49.5, 21), Math.toRadians(0))
                                .waitSeconds(pickUpDelay / 1000.0)
                                .strafeToLinearHeading(new Vector2d(-49.5, 45), Math.toRadians(310))
                                .strafeToLinearHeading(new Vector2d(-59, 21), Math.toRadians(0))
                                .waitSeconds(pickUpDelay / 1000.0)
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(-50, testYValue3 - 4, Math.toRadians(310)),
                                                Math.toRadians(90))
                                .waitSeconds(0.1)
                                .turnTo(Math.toRadians(90))
                                .strafeTo(new Vector2d(-50, testYValue3))
                                .waitSeconds(grabDelay / 1000.0)
                                .strafeToSplineHeading(new Vector2d(0, testYValue4), Math.toRadians(273))
                                .waitSeconds(clipDelay / 1000.0)
                                .strafeToLinearHeading(new Vector2d(-43, testYValue), Math.toRadians(90))
                                .waitSeconds(grabDelay / 1000.0)
                                .strafeToLinearHeading(new Vector2d(-2, testYValue4), Math.toRadians(273))
                                .waitSeconds(clipDelay / 1000.0)
                                .strafeToLinearHeading(new Vector2d(-43, testYValue), Math.toRadians(90))
                                .waitSeconds(grabDelay / 1000.0)
                                .strafeToLinearHeading(new Vector2d(-4, testYValue4), Math.toRadians(273))
                                .waitSeconds(clipDelay / 1000.0)
                                .strafeToLinearHeading(new Vector2d(-43, testYValue), Math.toRadians(90))
                                .waitSeconds(grabDelay / 1000.0)
                                .strafeToLinearHeading(new Vector2d(-6, testYValue4), Math.toRadians(273))
                                .waitSeconds(clipDelay / 1000.0)
                                .build();
        }

        // Legacy paths below this point
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
