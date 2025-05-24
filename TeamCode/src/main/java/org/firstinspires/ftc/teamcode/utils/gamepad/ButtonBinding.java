package org.firstinspires.ftc.teamcode.utils.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.commands.base.Command;
import org.firstinspires.ftc.teamcode.commands.base.CommandScheduler;

import java.util.function.Supplier;

public class ButtonBinding {
    private final ButtonEnum button;
    private final GamepadInteractionType interactionType;
    private final Supplier<Command> commandSupplier;
    private Command activeCommand;

    // Threshold for considering a trigger as "pressed"
    private static final float TRIGGER_THRESHOLD = 0.1f;

    public ButtonBinding(ButtonEnum button, GamepadInteractionType interactionType, Supplier<Command> commandSupplier) {
        this.button = button;
        this.interactionType = interactionType;
        this.commandSupplier = commandSupplier;
        this.activeCommand = null;
    }

    private boolean isButtonPressed(Gamepad gamepadState) {
        switch (button) {
            case A: return gamepadState.a;
            case B: return gamepadState.b;
            case X: return gamepadState.x;
            case Y: return gamepadState.y;
            case LEFT_BUMPER: return gamepadState.left_bumper;
            case RIGHT_BUMPER: return gamepadState.right_bumper;
            case DPAD_UP: return gamepadState.dpad_up;
            case DPAD_DOWN: return gamepadState.dpad_down;
            case DPAD_LEFT: return gamepadState.dpad_left;
            case DPAD_RIGHT: return gamepadState.dpad_right;
            case LEFT_STICK_BUTTON: return gamepadState.left_stick_button;
            case RIGHT_STICK_BUTTON: return gamepadState.right_stick_button;
            case LEFT_TRIGGER: return gamepadState.left_trigger > TRIGGER_THRESHOLD;
            case RIGHT_TRIGGER: return gamepadState.right_trigger > TRIGGER_THRESHOLD;
            case START: return gamepadState.start;
            case BACK: return gamepadState.back;
            default: return false;
        }
    }

    public void process(Gamepad currentGamepadState, Gamepad previousGamepadState, CommandScheduler scheduler) {
        boolean buttonPressedNow = isButtonPressed(currentGamepadState);
        boolean buttonPressedPreviously = isButtonPressed(previousGamepadState);

        switch (interactionType) {
            case WHEN_PRESSED:
                if (buttonPressedNow && !buttonPressedPreviously) {
                    scheduler.schedule(commandSupplier.get());
                }
                break;
            case WHEN_RELEASED:
                if (!buttonPressedNow && buttonPressedPreviously) {
                    scheduler.schedule(commandSupplier.get());
                }
                break;
            case WHILE_HELD:
                if (buttonPressedNow && !buttonPressedPreviously) {
                    activeCommand = commandSupplier.get();
                    if (activeCommand != null) {
                        scheduler.schedule(activeCommand);
                    }
                } else if (!buttonPressedNow && buttonPressedPreviously) {
                    if (activeCommand != null) {
                        scheduler.cancel(activeCommand);
                        activeCommand = null;
                    }
                }
                break;
        }
    }
}
