package org.firstinspires.ftc.teamcode.commands.slide;

import org.firstinspires.ftc.teamcode.commands.base.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.base.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.base.WaitCommand;
import org.firstinspires.ftc.teamcode.utils.ClawController;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

/**
 * Command for the lower+upper slide transfer sequence (auto/teleop)
 */
public class UpperSlideScoreCommand extends SequentialCommandGroup {

        public UpperSlideScoreCommand(UpperSlideCommands upperSlideCommands) {
                super();

                addCommands(
                        new ActionCommand(upperSlideCommands.inter()),
                        new ActionCommand(upperSlideCommands.slidePos3()),
                        new WaitCommand(ConfigVariables.UpperSlideVars.SLIDEPOS3_DELAY),
                        new ActionCommand(upperSlideCommands.front()),
                        new ActionCommand(upperSlideCommands.openExtendoClaw()),
                        new WaitCommand(ConfigVariables.UpperSlideVars.FRONT_DELAY),
                        new ActionCommand(upperSlideCommands.openClaw()),
                        new WaitCommand(ConfigVariables.General.CLAW_OPERATION_TIMEOUT),
                        new ActionCommand(upperSlideCommands.inter()),
                        new ActionCommand(upperSlideCommands.closeExtendoClaw())
                );
        }
}