package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class PrimaryMeepMeep {
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
    public static double botWidth =  13.386;
    public static double TRACK_WIDTH = 11.25286365;

    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(700);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 70, 55, 60, TRACK_WIDTH)
                .setDimensions(botWidth, botLength)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-6.5, 63, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(-8, testYValue2))
                .waitSeconds(clipDelay / 1000.0)
                .strafeTo(new Vector2d(-32, testYValue2))
                .strafeTo(new Vector2d(-48.5, 12.5))
                .waitSeconds(pickUpDelay / 1000.0)
                .strafeToLinearHeading(new Vector2d(-48.5, 45), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-48.5, 12.5), Math.toRadians(-90))
                .waitSeconds(pickUpDelay / 1000.0)
                .strafeToLinearHeading(new Vector2d(-58, 12.5), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-58, 45), Math.toRadians(-90))
                .waitSeconds(pickUpDelay / 1000.0)
                .strafeToLinearHeading(new Vector2d(-63, 12.5), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-63, 45), Math.toRadians(-90))
                .waitSeconds(pickUpDelay / 1000.0)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-41.5, testYValue3 - 4), Math.toRadians(310))
                .waitSeconds(0.1)
                .strafeTo(new Vector2d(-41.5, testYValue3))
                .waitSeconds(grabDelay / 1000.0)
                .strafeToSplineHeading(new Vector2d(0, testYValue4), Math.toRadians(270))
                .waitSeconds(clipDelay / 1000.0)
                .strafeToLinearHeading(new Vector2d(-43, testYValue), Math.toRadians(270))
                .waitSeconds(grabDelay / 1000.0)
                .strafeToLinearHeading(new Vector2d(-2, testYValue4), Math.toRadians(270))
                .waitSeconds(clipDelay / 1000.0)
                .strafeToLinearHeading(new Vector2d(-43, testYValue), Math.toRadians(270))
                .waitSeconds(grabDelay / 1000.0)
                .strafeToLinearHeading(new Vector2d(-4, testYValue4), Math.toRadians(270))
                .waitSeconds(clipDelay / 1000.0)
                .strafeToLinearHeading(new Vector2d(-43, testYValue), Math.toRadians(270))
                .waitSeconds(grabDelay / 1000.0)
                .strafeToLinearHeading(new Vector2d(-6, testYValue4), Math.toRadians(270))
                .waitSeconds(clipDelay / 1000.0)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}