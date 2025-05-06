package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class AutoPaths {
        // Store the starting poses for different paths
        public static final Pose2d START_POSE = new Pose2d(0, 0, 0);

        // Method to get the original test path
        public static Action getOriginalTestPath(MecanumDrive drive) {
                return drive.actionBuilder(START_POSE)
                                .splineTo(new Vector2d(30, 30), Math.PI / 2)
                                .splineTo(new Vector2d(0, 60), Math.PI)
                                .build();
        }

        // Method to get the square path
        public static Action getSquarePath(MecanumDrive drive) {
                return drive.actionBuilder(START_POSE)
                                .splineTo(new Vector2d(30, 0), 0)
                                .splineTo(new Vector2d(30, 30), Math.PI / 2)
                                .splineTo(new Vector2d(0, 30), Math.PI)
                                .splineTo(new Vector2d(0, 0), -Math.PI / 2)
                                .build();
        }

        // Method to get the chicane path
        public static Action getChicanePath(MecanumDrive drive) {
                return drive.actionBuilder(START_POSE)
                                .splineTo(new Vector2d(20, 20), Math.PI / 4)
                                .splineTo(new Vector2d(40, 0), -Math.PI / 4)
                                .splineTo(new Vector2d(60, 20), Math.PI / 4)
                                .build();
        }
}
