package com.example.meepmeeptesting.paths;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.roadrunner.DriveShim;

/**
 * Shared autonomous path definitions for both robot code and MeepMeep
 * simulation
 */
public final class AutoPaths {
    // Robot dimensions (in inches)
    public static final double BOT_LENGTH = 15.748;
    public static final double BOT_WIDTH = 13.386;
    public static final double TRACK_WIDTH = 11.25286365;

    // Common robot positions
    public static final class RobotPosition {
        public final Vector2d pos;
        public final double heading;
        public final Pose2d pose;

        public RobotPosition(double x, double y, double headingDegrees) {
            this.pos = new Vector2d(x, y);
            this.heading = Math.toRadians(headingDegrees);
            this.pose = new Pose2d(x, y, this.heading);
        }
    }

    // Sample autonomous positions
    public static final RobotPosition START = new RobotPosition(39.5, 65, 180);
    public static final RobotPosition PREPLACED = new RobotPosition(47, 46, -90); //?

    public static final RobotPosition PICKUP1 = new RobotPosition(56, 52, -110);
    public static final RobotPosition PICKUP2 = new RobotPosition(58, 55, -90);
    public static final RobotPosition PICKUP3 = new RobotPosition(47.1, 47.1, -45);
    public static final RobotPosition SCORE = new RobotPosition(57, 57, 225);



    // Primary autonomous path points
    public static final double TEST_Y_VALUE = 61.5;
    public static final double TEST_Y_VALUE2 = 33;
    public static final double TEST_Y_VALUE3 = 61.5;
    public static final double TEST_Y_VALUE4 = 33;
    public static final double THIRD_SPECIMEN_OFFSET = 3.5;
    public static final double FOURTH_SPECIMEN_OFFSET = 3.5;
    public static final double CLIP_OFFSET = 3.5;
    public static final double TEST_X_VALUE = -45;

    // Timing constants (in milliseconds)
    public static final int CLIP_DELAY = 200;
    public static final int GRAB_DELAY = 100;
    public static final int PICKUP_DELAY = 200;
    public static final int DROP_OFF_DELAY = 200;
    public static final double EXTEND_DELAY = 1;
    public static final int EXTEND_LENGTH = 515;

    // Robot control parameters
    public static final double NEUTRAL_PITCH = 0.15;
    public static final double NEUTRAL_YAW = 1;

    // Drive constraints
    public static final double MAX_VEL = 90;
    public static final double MAX_ACCEL = 70;
    public static final double MAX_ANG_VEL = 55;
    public static final double MAX_ANG_ACCEL = 60;


    // Helper method to reset robot pose
    public static void resetPose(DriveShim drive) {
        drive.setPoseEstimate(START.pose);
    }



    public static TrajectoryActionBuilder autosamplepath(DriveShim drive) {
        return drive.actionBuilder(START.pose)
                .strafeToSplineHeading(SCORE.pos, SCORE.heading-Math.toRadians(10))
                .strafeToSplineHeading(PICKUP1.pos, PICKUP1.heading)
                .strafeToSplineHeading(SCORE.pos, SCORE.heading+Math.toRadians(20))
                .strafeToSplineHeading(PICKUP2.pos, PICKUP2.heading)
                .strafeToSplineHeading(SCORE.pos, SCORE.heading-Math.toRadians(10))
                .strafeToSplineHeading(PICKUP3.pos, PICKUP3.heading)
                .strafeToSplineHeading(SCORE.pos, SCORE.heading+Math.toRadians(20))


                //tank path
                .strafeTo(new Vector2d(39,28))
                .splineTo(new Vector2d(30,15), Math.toRadians(205))

                .setReversed(true)
                .splineTo(new Vector2d(44,28), SCORE.heading-Math.toRadians(170))
                .splineTo(SCORE.pos, SCORE.heading-Math.toRadians(170));

    }



    public static TrajectoryActionBuilder getHangFirstPath(DriveShim drive){
        return drive.actionBuilder(START.pose)
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
        return drive.actionBuilder(START.pose)
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
        return drive.actionBuilder(START.pose)
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
        return drive.actionBuilder(START.pose)
                .splineTo(new Vector2d(30, 30), Math.PI / 2)
                .splineTo(new Vector2d(0, 60), Math.PI)
                .build();
    }

    // Method to get the square path
    public static Action getSquarePath(DriveShim drive) {
        return drive.actionBuilder(START.pose)
                .splineTo(new Vector2d(30, 0), 0)
                .splineTo(new Vector2d(30, 30), Math.PI / 2)
                .splineTo(new Vector2d(0, 30), Math.PI)
                .splineTo(new Vector2d(0, 0), -Math.PI / 2)
                .build();
    }

    // Method to get the chicane path
    public static Action getChicanePath(DriveShim drive) {
        return drive.actionBuilder(START.pose)
                .splineTo(new Vector2d(20, 20), Math.PI / 4)
                .splineTo(new Vector2d(40, 0), -Math.PI / 4)
                .splineTo(new Vector2d(60, 20), Math.PI / 4)
                .build();
    }
}
