package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.example.meepmeeptesting.paths.AutoPaths;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SecondaryMeepMeep {

    public static double botLength = 15.748;
    public static double botWidth =  13.386;
    public static double TRACK_WIDTH = 11.25286365;

    public static void main(String[] args) throws InterruptedException{
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(700);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), TRACK_WIDTH)
                .setDimensions(botWidth, botLength)
                .build();

        myBot.runAction(AutoPaths.autosamplepath(myBot.getDrive()).build());
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(30.5, 62, Math.toRadians(270)))

//                .strafeToLinearHeading(new Vector2d(47, 46), Math.toRadians(-90))
//
//                .strafeToLinearHeading(new Vector2d(60, 60), Math.toRadians(225))
//                .strafeToConstantHeading(new Vector2d(55, 55))
//
//                .strafeToLinearHeading(new Vector2d(58,46), Math.toRadians(-90))
//                .strafeToLinearHeading(new Vector2d(60, 60), Math.toRadians(225))
//                .strafeToConstantHeading(new Vector2d(55, 55))
//
//                .strafeToLinearHeading(new Vector2d(58,46), Math.toRadians(-55))
//                .strafeToLinearHeading(new Vector2d(60, 60), Math.toRadians(225))
//                .strafeToConstantHeading(new Vector2d(55, 55))
//
//                .strafeToLinearHeading(new Vector2d(38,32), Math.toRadians(180))
//
//
//                .strafeToConstantHeading(new Vector2d(38,12))
//                .strafeToConstantHeading(new Vector2d(23,10))
//
//                .strafeToConstantHeading(new Vector2d(38,12))
//                .strafeToLinearHeading(new Vector2d(60, 60), Math.toRadians(225))
//                .strafeToConstantHeading(new Vector2d(55, 55))
//                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}