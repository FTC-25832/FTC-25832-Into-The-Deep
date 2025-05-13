package com.example.meepmeeptesting;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import static com.example.meepmeeptesting.paths.AutoPaths.*;

public class PrimaryMeepMeep {
        public static void main(String[] args) {
                System.setProperty("sun.java2d.opengl", "true");
                MeepMeep meepMeep = new MeepMeep(700);

                RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                                // Set bot constraints from shared constants
                                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                                .setDimensions(BOT_WIDTH, BOT_LENGTH)
                                .build();

                myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-6.5, 63, Math.toRadians(-90)))
                                .strafeToConstantHeading(new Vector2d(-8, TEST_Y_VALUE2))
                                .waitSeconds(CLIP_DELAY / 1000.0)
                                .strafeTo(new Vector2d(-32, TEST_Y_VALUE2))
                                .strafeTo(new Vector2d(-48.5, 12.5))
                                .waitSeconds(PICKUP_DELAY / 1000.0)
                                .strafeToLinearHeading(new Vector2d(-48.5, 45), Math.toRadians(-90))
                                .strafeToLinearHeading(new Vector2d(-48.5, 12.5), Math.toRadians(-90))
                                .waitSeconds(PICKUP_DELAY / 1000.0)
                                .strafeToLinearHeading(new Vector2d(-58, 12.5), Math.toRadians(-90))
                                .strafeToLinearHeading(new Vector2d(-58, 45), Math.toRadians(-90))
                                .waitSeconds(PICKUP_DELAY / 1000.0)
                                .strafeToLinearHeading(new Vector2d(-63, 12.5), Math.toRadians(-90))
                                .strafeToLinearHeading(new Vector2d(-63, 45), Math.toRadians(-90))
                                .waitSeconds(PICKUP_DELAY / 1000.0)
                                .setTangent(Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-41.5, TEST_Y_VALUE3 - 4), Math.toRadians(310))
                                .waitSeconds(0.1)
                                .strafeTo(new Vector2d(-41.5, TEST_Y_VALUE3))
                                .waitSeconds(GRAB_DELAY / 1000.0)
                                .strafeToSplineHeading(new Vector2d(0, TEST_Y_VALUE4), Math.toRadians(270))
                                .waitSeconds(CLIP_DELAY / 1000.0)
                                .strafeToLinearHeading(new Vector2d(-43, TEST_Y_VALUE), Math.toRadians(270))
                                .waitSeconds(GRAB_DELAY / 1000.0)
                                .strafeToLinearHeading(new Vector2d(-2, TEST_Y_VALUE4), Math.toRadians(270))
                                .waitSeconds(CLIP_DELAY / 1000.0)
                                .strafeToLinearHeading(new Vector2d(-43, TEST_Y_VALUE), Math.toRadians(270))
                                .waitSeconds(GRAB_DELAY / 1000.0)
                                .strafeToLinearHeading(new Vector2d(-4, TEST_Y_VALUE4), Math.toRadians(270))
                                .waitSeconds(CLIP_DELAY / 1000.0)
                                .strafeToLinearHeading(new Vector2d(-43, TEST_Y_VALUE), Math.toRadians(270))
                                .waitSeconds(GRAB_DELAY / 1000.0)
                                .strafeToLinearHeading(new Vector2d(-6, TEST_Y_VALUE4), Math.toRadians(270))
                                .waitSeconds(CLIP_DELAY / 1000.0)
                                .build());

                meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                                .setDarkMode(true)
                                .setBackgroundAlpha(0.95f)
                                .addEntity(myBot)
                                .start();
        }
}
