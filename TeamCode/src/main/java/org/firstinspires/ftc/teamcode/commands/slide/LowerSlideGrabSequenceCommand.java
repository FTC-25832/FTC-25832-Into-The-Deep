package org.firstinspires.ftc.teamcode.commands.slide;

import org.firstinspires.ftc.teamcode.commands.base.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.base.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.base.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

public class LowerSlideGrabSequenceCommand extends SequentialCommandGroup {
        public LowerSlideGrabSequenceCommand(LowerSlide lowSlide) {
                super(); // 调用父类的无参构造函数
                addCommands(
                        new ActionCommand(new LowerSlideCommands(lowSlide).openClaw()),
                        new ActionCommand(new LowerSlideCommands(lowSlide).grabPart1()),
                        new ActionCommand(new LowerSlideCommands(lowSlide).grabPart2()),
                        new WaitCommand(ConfigVariables.LowerSlideVars.POS_GRAB_TIMEOUT / 1000.0),
                        new ActionCommand(new LowerSlideCommands(lowSlide).closeClaw()),
                        new WaitCommand(ConfigVariables.LowerSlideVars.CLAW_CLOSE_TIMEOUT / 1000.0),
                        new ActionCommand(new LowerSlideCommands(lowSlide).hover())
                );
        }
}