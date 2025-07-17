package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;

public class RobotStateStore {
    private static Pose2d pose = new Pose2d(0, 0, 0);
    private static int lowerSlideTick = 0;
    private static int upperSlideTick = 0;

    public static void loadSlides(LowerSlide lowslide, UpperSlide upslide) {
        lowslide.setTickOffset(lowerSlideTick);
        upslide.setTickOffset(upperSlideTick);
    }

    public static void save(Pose2d currentpose, int lowslidetick, int upslidetick) {
        pose = currentpose;
        lowerSlideTick = lowslidetick;
        upperSlideTick = upslidetick;
    }

    public static Pose2d getPose() {
        return pose;
    }
}
