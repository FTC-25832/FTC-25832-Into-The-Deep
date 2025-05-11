package org.firstinspires.ftc.teamcode.commands.base;

import com.acmerobotics.roadrunner.Action;

public interface ActionConvertable {
        /**
         * Convert this command to a RoadRunner Action
         */
        Action toAction();
}
