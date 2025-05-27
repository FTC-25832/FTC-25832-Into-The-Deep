package com.example.meepmeeptesting.paths;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.roadrunner.DriveShim;

import com.example.meepmeeptesting.shared.RobotState;
import com.example.meepmeeptesting.shared.MovementConstants;
import com.example.meepmeeptesting.shared.TimingConstants;

/**
 * Shared autonomous path definitions for both robot code and MeepMeep
 * simulation.
 * Uses structured state management and constants.
 */
public final class AutoPaths {
    // Constants referenced from shared classes
    public static final double BOT_LENGTH = MovementConstants.RobotDimensions.BOT_LENGTH;
    public static final double BOT_WIDTH = MovementConstants.RobotDimensions.BOT_WIDTH;
    public static final double TRACK_WIDTH = MovementConstants.RobotDimensions.TRACK_WIDTH;
    public static final double MAX_VEL = MovementConstants.DriveConstraints.MAX_VEL;
    public static final double MAX_ACCEL = MovementConstants.DriveConstraints.MAX_ACCEL;
    public static final double MAX_ANG_VEL = MovementConstants.DriveConstraints.MAX_ANG_VEL;
    public static final double MAX_ANG_ACCEL = MovementConstants.DriveConstraints.MAX_ANG_ACCEL;

    // Helper method to reset robot pose
    public static void resetPose(DriveShim drive) {
        drive.setPoseEstimate(RobotState.START.getPose());
    }

    /**
     * Build autonomous path with proper state handling and phase transitions
     */
    public static TrajectoryActionBuilder buildMainPath(DriveShim drive) {
        TrajectoryActionBuilder builder = drive.actionBuilder(RobotState.START.getPose());

        // Score preloaded pixel
        addScoreSequence(builder, RobotState.SCORE_POSITION);

        // First pickup and score
        addPickupSequence(builder, RobotState.PICKUP_1);
        addScoreSequence(builder, RobotState.SCORE_POSITION);

        // Second pickup and score
        addPickupSequence(builder, RobotState.PICKUP_2);
        addScoreSequence(builder, RobotState.SCORE_POSITION);

        // Third pickup and score
        addPickupSequence(builder, RobotState.PICKUP_3);
        addScoreSequence(builder, RobotState.SCORE_POSITION);

        // Transition to endgame
        addEndgameSequence(builder);

        return builder;
    }

    /**
     * Add a scoring sequence with approach and retreat
     */
    private static void addScoreSequence(TrajectoryActionBuilder builder, RobotState scoreState) {
        // Approach scoring position
        builder.setReversed(true)
                .splineToLinearHeading(scoreState.getApproachState().getPose(),
                        scoreState.getApproachState().getHeadingRadians())
                .splineToLinearHeading(scoreState.getPose(),
                        scoreState.getHeadingRadians())
                .setReversed(false);

        // Retreat from scoring position
        builder.splineToLinearHeading(scoreState.getApproachState().getPose(),
                scoreState.getApproachState().getHeadingRadians());
    }

    /**
     * Add a pickup sequence with approach
     */
    private static void addPickupSequence(TrajectoryActionBuilder builder, RobotState pickupState) {
        builder.setReversed(true)
                .splineToLinearHeading(pickupState.getApproachState().getPose(),
                        pickupState.getApproachState().getHeadingRadians())
                .splineToLinearHeading(pickupState.getPose(),
                        pickupState.getHeadingRadians())
                .setReversed(false);
    }

    /**
     * Add endgame sequence for hanging
     */
    private static void addEndgameSequence(TrajectoryActionBuilder builder) {
        // Approach hang position
        builder.splineToLinearHeading(RobotState.TANK_APPROACH.getPose(),
                RobotState.TANK_APPROACH.getHeadingRadians())
                .splineToLinearHeading(RobotState.TANK_FINAL.getPose(),
                        RobotState.TANK_FINAL.getHeadingRadians())
                .splineToLinearHeading(RobotState.HANG_START.getPose(),
                        RobotState.HANG_START.getHeadingRadians())
                .splineToLinearHeading(RobotState.HANG_APPROACH.getPose(),
                        RobotState.HANG_APPROACH.getHeadingRadians())
                .turn(Math.toRadians(-120));
    }

    /**
     * Build hang-first autonomous path
     */
    public static TrajectoryActionBuilder buildHangFirstPath(DriveShim drive) {
        return drive.actionBuilder(RobotState.START.getPose())
                .splineToLinearHeading(RobotState.HANG_START.getPose(),
                        RobotState.HANG_START.getHeadingRadians())
                .splineToLinearHeading(RobotState.HANG_APPROACH.getPose(),
                        RobotState.HANG_APPROACH.getHeadingRadians());
    }

    private AutoPaths() {
        // Private constructor to prevent instantiation
    }
}
