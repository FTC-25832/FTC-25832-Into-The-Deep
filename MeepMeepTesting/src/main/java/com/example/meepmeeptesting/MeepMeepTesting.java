package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import com.example.meepmeeptesting.paths.AutoPaths;

public class MeepMeepTesting {
    public static void main(String[] args) {
        double TRACK_WIDTH = 28.2797;
        double maxWheelVel = 240;
        double minProfileAccel = -60;
        double maxProfileAccel = 240;
        double maxAngVel = Math.PI;
        double maxAngAccel = Math.PI;

        double botLength = 15.748;
        double botWidth =  13.386;
//        default meepmeep settings are Bot Width: 18in
//          Bot Height: 18in



        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity ourBot =  new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(maxWheelVel, maxProfileAccel, maxAngVel, maxAngAccel, TRACK_WIDTH)
                .build();



//
//        AutoPaths.resetPose(ourBot.getDrive());
//        ourBot.runAction(AutoPaths.getOriginalTestPath(ourBot.getDrive()));
//
//
//        AutoPaths.resetPose(ourBot.getDrive());
//        ourBot.runAction(AutoPaths.getSquarePath(ourBot.getDrive()));
//

        AutoPaths.resetPose(ourBot.getDrive());
        ourBot.runAction(AutoPaths.getPrePlaced(ourBot.getDrive()));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(ourBot)
                .start();
    }
}
