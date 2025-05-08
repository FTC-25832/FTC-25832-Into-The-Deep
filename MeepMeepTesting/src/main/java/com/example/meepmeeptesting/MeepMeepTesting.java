package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import com.example.meepmeeptesting.paths.AutoPaths;

public class MeepMeepTesting {
    public static void main(String[] args) {
        double TRACK_WIDTH = 11.25286365;
        double maxWheelVel = 50;
        double minProfileAccel = -45;
        double maxProfileAccel = 50;
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
                .setDimensions(botWidth, botLength)
                .setDriveTrainType(DriveTrainType.MECANUM)
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
//        ourBot.runAction(AutoPaths.getPrePlaced(ourBot.getDrive()));
        Action trajectoryActionChosen;
//        if (startPosition == 1) {
//            trajectoryActionChosen = tab1.build();
//        } else if (startPosition == 2) {
//            trajectoryActionChosen = tab2.build();
//        } else {
//            trajectoryActionChosen = tab3.build();
//        }
//        trajectoryActionChosen = AutoPaths.getAutoPaths(ourBot.getDrive()).build();
//       use if starposition depending on where robot is for choice of auto? idk

        ourBot.runAction(
                new SequentialAction(
                        AutoPaths.getHangFirstPath(ourBot.getDrive()).build(),
                        AutoPaths.getGotoPreplaced(ourBot.getDrive()).build(),

                        //extend lowerslide to get furtherst
                        //claw pickup
                        AutoPaths.getRotateToTeamBox(ourBot.getDrive()).build(),
                        //first block dropoff complete
                        
                        //second block sequence (x = -52)
                        AutoPaths.getGotoPreplacedSecond(ourBot.getDrive()).build(),
                        AutoPaths.getRotateToTeamBoxSecond(ourBot.getDrive()).build(),
                        
                        //third block sequence (x = -61)
                        AutoPaths.getGotoPreplacedThird(ourBot.getDrive()).build(),
                        AutoPaths.getRotateToTeamBoxThird(ourBot.getDrive()).build(),

                        //rotate to get specimen to hand

                        AutoPaths.getStart(ourBot.getDrive()).build(),
                        //pickup

                        //start handing sequence
                        AutoPaths.getGoToSpecimenHang(ourBot.getDrive()).build(),
                        AutoPaths.getGoToTeamBox(ourBot.getDrive()).build(),

                        //go to there, hang
                        //go back
                        //get
                        //repeat
                        AutoPaths.getGoToSpecimenHang(ourBot.getDrive()).build(),
                        AutoPaths.getGoToTeamBox(ourBot.getDrive()).build(),
                        AutoPaths.getGoToSpecimenHang(ourBot.getDrive()).build(),
                        AutoPaths.getGoToTeamBox(ourBot.getDrive()).build(),
                        AutoPaths.getGoToSpecimenHang(ourBot.getDrive()).build(),
                        AutoPaths.getGoToTeamBox(ourBot.getDrive()).build()


//                        lift.liftUp(),
//                        claw.openClaw(),
//                        lift.liftDown(),
//                        trajectoryActionCloseOut
                )
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(ourBot)
                .start();
    }
}
