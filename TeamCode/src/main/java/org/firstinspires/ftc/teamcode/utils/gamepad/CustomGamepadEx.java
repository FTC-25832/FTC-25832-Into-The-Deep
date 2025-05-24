package org.firstinspires.ftc.teamcode.utils.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.commands.base.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.base.Command;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class CustomGamepadEx {
    private final Gamepad gamepad; // The underlying FTC gamepad
    private final Gamepad previousGamepadState; // Stores the state of the gamepad in the previous loop cycle
    private final List<ButtonBinding> bindings;
    private final CommandScheduler scheduler;

    // Threshold for considering a trigger as "pressed"
    private static final float TRIGGER_THRESHOLD = 0.1f;

    public CustomGamepadEx(Gamepad gamepad, CommandScheduler scheduler) {
        this.gamepad = gamepad;
        this.scheduler = scheduler;
        this.previousGamepadState = new Gamepad(); // Initialize with a new Gamepad object
        this.bindings = new ArrayList<>();
        // Initialize previousGamepadState with current state once to avoid null issues on first frame
        copyGamepadState(this.gamepad, this.previousGamepadState);
    }

    public ButtonExpose button(ButtonEnum button) {
        return new ButtonExpose(this, button);
    }

    void addBinding(ButtonBinding binding) {
        this.bindings.add(binding);
    }

    public void update() {
        // Create a temporary Gamepad object to hold the true current state for this cycle's processing
        Gamepad currentGamepadForProcessing = new Gamepad();
        copyGamepadState(this.gamepad, currentGamepadForProcessing);

        for (ButtonBinding binding : bindings) {
            // Pass the true current state and the state from the previous update
            binding.process(currentGamepadForProcessing, previousGamepadState, scheduler);
        }

        // After processing all bindings, update previousGamepadState to reflect the state of this.gamepad
        // for the next iteration.
        copyGamepadState(this.gamepad, previousGamepadState);
    }

    // Manually copy relevant gamepad fields
    private void copyGamepadState(Gamepad source, Gamepad destination) {
        destination.a = source.a;
        destination.b = source.b;
        destination.x = source.x;
        destination.y = source.y;
        destination.left_bumper = source.left_bumper;
        destination.right_bumper = source.right_bumper;
        destination.dpad_up = source.dpad_up;
        destination.dpad_down = source.dpad_down;
        destination.dpad_left = source.dpad_left;
        destination.dpad_right = source.dpad_right;
        destination.left_stick_button = source.left_stick_button;
        destination.right_stick_button = source.right_stick_button;
        destination.left_trigger = source.left_trigger;
        destination.right_trigger = source.right_trigger;
        destination.start = source.start;
        destination.back = source.back;

        // Copy other fields if they become relevant for ButtonEnum
        // For example:
        // destination.left_stick_x = source.left_stick_x;
        // destination.left_stick_y = source.left_stick_y;
        // destination.right_stick_x = source.right_stick_x;
        // destination.right_stick_y = source.right_stick_y;
        // ... and any other fields used by Gamepad.
    }

    // Inner class ButtonExpose (Step 5)
    public static class ButtonExpose {
        private final CustomGamepadEx gamepadEx;
        private final ButtonEnum button;

        public ButtonExpose(CustomGamepadEx gamepadEx, ButtonEnum button) {
            this.gamepadEx = gamepadEx;
            this.button = button;
        }

        public CustomGamepadEx whenPressed(Supplier<Command> commandSupplier) {
            gamepadEx.addBinding(new ButtonBinding(button, GamepadInteractionType.WHEN_PRESSED, commandSupplier));
            return gamepadEx;
        }

        public CustomGamepadEx whenReleased(Supplier<Command> commandSupplier) {
            gamepadEx.addBinding(new ButtonBinding(button, GamepadInteractionType.WHEN_RELEASED, commandSupplier));
            return gamepadEx;
        }

        public CustomGamepadEx whileHeld(Supplier<Command> commandSupplier) {
            gamepadEx.addBinding(new ButtonBinding(button, GamepadInteractionType.WHILE_HELD, commandSupplier));
            return gamepadEx;
        }
    }
}
