package com.example.meepmeeptesting.paths;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.roadrunner.DriveShim;

public class AutoPaths {
    // Store the starting poses for different paths
    public static final Pose2d START_POSE = new Pose2d(0, 62.5, -Math.PI / 2);

    // Helper method to reset robot pose
    public static void resetPose(DriveShim drive) {
        drive.setPoseEstimate(START_POSE);
    }


    public static TrajectoryActionBuilder getHangFirstPath(DriveShim drive){
        return drive.actionBuilder(START_POSE)
                .splineToConstantHeading(new Vector2d(0, 30), -Math.PI / 2);
    }

    // First block (x = -40.5)
    public static TrajectoryActionBuilder getGotoPreplaced(DriveShim drive){
        return drive.actionBuilder(new Pose2d(0, 30, -Math.PI / 2))
            .strafeToSplineHeading(new Vector2d(-40.5, 40.5), Math.toRadians(-135));
    }
    public static TrajectoryActionBuilder getRotateToTeamBox(DriveShim drive){
        return drive.actionBuilder(new Pose2d(-40.5, 40.5, Math.toRadians(-135)))
                .turn(Math.toRadians(-120)); // -45° - 75°
    }



    // Second block (x = -52)
    public static TrajectoryActionBuilder getGotoPreplacedSecond(DriveShim drive){
        return drive.actionBuilder(new Pose2d(-40.5, 40.5, Math.toRadians(105)))
                .splineTo(new Vector2d(-52, 40.5), Math.toRadians(-135));
    }
    public static TrajectoryActionBuilder getRotateToTeamBoxSecond(DriveShim drive){
        return drive.actionBuilder(new Pose2d(-52, 40.5, Math.toRadians(-135)))
                .turn(Math.toRadians(-120));
    }


    // Third block (x = -61)
    public static TrajectoryActionBuilder getGotoPreplacedThird(DriveShim drive){
        return drive.actionBuilder(new Pose2d(-52, 40.5, Math.toRadians(105)))
                .splineTo(new Vector2d(-61, 40.5), Math.toRadians(-135));
    }
    public static TrajectoryActionBuilder getRotateToTeamBoxThird(DriveShim drive){
        return drive.actionBuilder(new Pose2d(-61, 40.5, Math.toRadians(-135)))
                .turn(Math.toRadians(-120)); // -45° - 75°
    }

    public static TrajectoryActionBuilder getStart(DriveShim drive){
        return drive.actionBuilder(new Pose2d(-61, 40.5, Math.toRadians(105)))
                .strafeToSplineHeading(new Vector2d(-47, 62.5), Math.toRadians(-90));
    }
    public static TrajectoryActionBuilder getGoToSpecimenHang(DriveShim drive){
        return drive.actionBuilder(new Pose2d(-47, 62.5, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(-10, 30), -Math.PI / 2);
    }
    public static TrajectoryActionBuilder getGoToTeamBox(DriveShim drive){
        return drive.actionBuilder(new Pose2d(-10, 30, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(-47, 62.5));
    }

    public static TrajectoryActionBuilder getAutoPaths(DriveShim drive) {
            return drive.actionBuilder(START_POSE)
//                .lineToYSplineHeading(33, Math.toRadians(0))
//                .waitSeconds(2)
//                .setTangent(Math.toRadians(90))
//                .lineToY(48)
//                .setTangent(Math.toRadians(0))
//                .lineToX(32)
//                .strafeTo(new Vector2d(44.5, 30))
//                .turn(Math.toRadians(180))
//                .lineToX(47.5)
//                .waitSeconds(3);
                    .splineToConstantHeading(new Vector2d(-51, 30), Math.PI / 2)
                    .splineToConstantHeading(new Vector2d(-51, 56), Math.PI / 2)
                    .splineToConstantHeading(new Vector2d(-60, 30), Math.PI / 2)
                    .splineToConstantHeading(new Vector2d(-60, 56), Math.PI / 2)
                    .splineToConstantHeading(new Vector2d(-65, 30), Math.PI / 2)
                    .splineToConstantHeading(new Vector2d(-65, 56), Math.PI / 2)

                    .splineToConstantHeading(new Vector2d(0, 30), Math.PI / 2)
                    .splineToConstantHeading(new Vector2d(-49, 56), Math.PI / 2)

                    .splineToConstantHeading(new Vector2d(0, 30), Math.PI / 2)
                    .splineToConstantHeading(new Vector2d(-49, 56), Math.PI / 2)
                    .splineToConstantHeading(new Vector2d(0, 30), Math.PI / 2)
                    .splineToConstantHeading(new Vector2d(-49, 56), Math.PI / 2)
                    .splineToConstantHeading(new Vector2d(0, 30), Math.PI / 2)
                    .splineToConstantHeading(new Vector2d(-49, 56), Math.PI / 2)

                    .waitSeconds(3);
    }

    public static Action getPrePlaced(DriveShim drive) {
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



    // Method to get the original test path
    public static Action getOriginalTestPath(DriveShim drive) {
        return drive.actionBuilder(START_POSE)
                .splineTo(new Vector2d(30, 30), Math.PI / 2)
                .splineTo(new Vector2d(0, 60), Math.PI)
                .build();
    }

    // Method to get the square path
    public static Action getSquarePath(DriveShim drive) {
        return drive.actionBuilder(START_POSE)
                .splineTo(new Vector2d(30, 0), 0)
                .splineTo(new Vector2d(30, 30), Math.PI / 2)
                .splineTo(new Vector2d(0, 30), Math.PI)
                .splineTo(new Vector2d(0, 0), -Math.PI / 2)
                .build();
    }

    // Method to get the chicane path
    public static Action getChicanePath(DriveShim drive) {
        return drive.actionBuilder(START_POSE)
                .splineTo(new Vector2d(20, 20), Math.PI / 4)
                .splineTo(new Vector2d(40, 0), -Math.PI / 4)
                .splineTo(new Vector2d(60, 20), Math.PI / 4)
                .build();
    }
}
