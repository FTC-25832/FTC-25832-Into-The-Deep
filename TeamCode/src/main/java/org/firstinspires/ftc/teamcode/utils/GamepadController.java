package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

/**
 * An enhanced controller for FTC Gamepad that provides event-based handling
 * for button presses and releases, along with other utility functions.
 */
public class GamepadController {
    // The gamepad to monitor
    private final Gamepad gamepad;

    // Maps to store the previous state of buttons
    private final Map<BooleanSupplier, Boolean> prevButtonStates = new HashMap<>();

    // Maps to store callbacks for button events
    private final Map<BooleanSupplier, Runnable> pressCallbacks = new HashMap<>();
    private final Map<BooleanSupplier, Runnable> releaseCallbacks = new HashMap<>();
    private final Map<BooleanSupplier, Consumer<Double>> whilePressedCallbacks = new HashMap<>();

    // Threshold for considering analog inputs as "pressed"
    private static final double DEFAULT_THRESHOLD = 0.5;
    private double threshold = DEFAULT_THRESHOLD;

    /**
     * Creates a new GamepadController for the given gamepad.
     *
     * @param gamepad The gamepad to monitor
     */
    public GamepadController(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    /**
     * Sets the threshold for considering analog inputs as "pressed".
     *
     * @param threshold The threshold value (0.0 to 1.0)
     * @return This controller for method chaining
     */
    public GamepadController setThreshold(double threshold) {
        this.threshold = Math.max(0.0, Math.min(1.0, threshold));
        return this;
    }

    /**
     * Registers a callback to be called when a button is pressed.
     *
     * @param buttonSupplier The function that returns the button state
     * @param callback The callback to execute when the button is pressed
     * @return This controller for method chaining
     */
    public GamepadController onPressed(BooleanSupplier buttonSupplier, Runnable callback) {
        pressCallbacks.put(buttonSupplier, callback);
        prevButtonStates.putIfAbsent(buttonSupplier, false);
        return this;
    }

    /**
     * Registers a callback to be called when a button is pressed.
     * Simplified version that takes a ButtonType directly.
     *
     * @param button The button type to monitor
     * @param callback The callback to execute when the button is pressed
     * @return This controller for method chaining
     */
    public GamepadController onPressed(ButtonType button, Runnable callback) {
        return onPressed(button(button), callback);
    }

    /**
     * Registers a callback to be called when a button is released.
     *
     * @param buttonSupplier The function that returns the button state
     * @param callback The callback to execute when the button is released
     * @return This controller for method chaining
     */
    public GamepadController onReleased(BooleanSupplier buttonSupplier, Runnable callback) {
        releaseCallbacks.put(buttonSupplier, callback);
        prevButtonStates.putIfAbsent(buttonSupplier, false);
        return this;
    }

    /**
     * Registers a callback to be called when a button is released.
     * Simplified version that takes a ButtonType directly.
     *
     * @param button The button type to monitor
     * @param callback The callback to execute when the button is released
     * @return This controller for method chaining
     */
    public GamepadController onReleased(ButtonType button, Runnable callback) {
        return onReleased(button(button), callback);
    }

    /**
     * Registers a callback to be called continuously while a button is pressed.
     *
     * @param buttonSupplier The function that returns the button state
     * @param callback The callback to execute while the button is pressed,
     *                 receives a value from 0.0 to 1.0 for analog inputs
     * @return This controller for method chaining
     */
    public GamepadController whilePressed(BooleanSupplier buttonSupplier, Consumer<Double> callback) {
        whilePressedCallbacks.put(buttonSupplier, callback);
        prevButtonStates.putIfAbsent(buttonSupplier, false);
        return this;
    }

    /**
     * Registers a callback to be called continuously while a button is pressed.
     * Simplified version that takes a ButtonType directly.
     *
     * @param button The button type to monitor
     * @param callback The callback to execute while the button is pressed
     * @return This controller for method chaining
     */
    public GamepadController whilePressed(ButtonType button, Consumer<Double> callback) {
        return whilePressed(button(button), callback);
    }

    /**
     * Updates the controller state and executes any registered callbacks.
     * Should be called once per loop cycle.
     */
    public void update() {
        // Check button press/release events
        for (Map.Entry<BooleanSupplier, Boolean> entry : prevButtonStates.entrySet()) {
            BooleanSupplier buttonSupplier = entry.getKey();
            boolean prevState = entry.getValue();
            boolean currentState = buttonSupplier.getAsBoolean();

            // Check for button press
            if (!prevState && currentState) {
                Runnable callback = pressCallbacks.get(buttonSupplier);
                if (callback != null) {
                    callback.run();
                }
            }

            // Check for button release
            if (prevState && !currentState) {
                Runnable callback = releaseCallbacks.get(buttonSupplier);
                if (callback != null) {
                    callback.run();
                }
            }

            // Check for while pressed
            if (currentState) {
                Consumer<Double> callback = whilePressedCallbacks.get(buttonSupplier);
                if (callback != null) {
                    callback.accept(1.0); // Default to 1.0 for boolean buttons
                }
            }

            // Update previous state
            entry.setValue(currentState);
        }
    }

    /**
     * Creates a supplier for a gamepad button.
     */
    public BooleanSupplier button(ButtonType button) {
        switch (button) {
            case A:
                return () -> gamepad.a;
            case B:
                return () -> gamepad.b;
            case X:
                return () -> gamepad.x;
            case Y:
                return () -> gamepad.y;
            case DPAD_UP:
                return () -> gamepad.dpad_up;
            case DPAD_DOWN:
                return () -> gamepad.dpad_down;
            case DPAD_LEFT:
                return () -> gamepad.dpad_left;
            case DPAD_RIGHT:
                return () -> gamepad.dpad_right;
            case LEFT_BUMPER:
                return () -> gamepad.left_bumper;
            case RIGHT_BUMPER:
                return () -> gamepad.right_bumper;
            case LEFT_STICK_BUTTON:
                return () -> gamepad.left_stick_button;
            case RIGHT_STICK_BUTTON:
                return () -> gamepad.right_stick_button;
            case BACK:
                return () -> gamepad.back;
            case START:
                return () -> gamepad.start;
            case GUIDE:
                return () -> gamepad.guide;
            default:
                return () -> false;
        }
    }

    /**
     * Creates a supplier for a gamepad trigger.
     */
    public BooleanSupplier trigger(TriggerType trigger) {
        switch (trigger) {
            case LEFT_TRIGGER:
                return () -> gamepad.left_trigger > threshold;
            case RIGHT_TRIGGER:
                return () -> gamepad.right_trigger > threshold;
            default:
                return () -> false;
        }
    }

    /**
     * Gets the raw value of a trigger (0.0 to 1.0).
     */
    public double getTriggerValue(TriggerType trigger) {
        switch (trigger) {
            case LEFT_TRIGGER:
                return gamepad.left_trigger;
            case RIGHT_TRIGGER:
                return gamepad.right_trigger;
            default:
                return 0.0;
        }
    }

    /**
     * Gets the X value of a joystick (-1.0 to 1.0).
     */
    public double getJoystickX(JoystickType joystick) {
        switch (joystick) {
            case LEFT:
                return gamepad.left_stick_x;
            case RIGHT:
                return gamepad.right_stick_x;
            default:
                return 0.0;
        }
    }

    /**
     * Gets the Y value of a joystick (-1.0 to 1.0).
     */
    public double getJoystickY(JoystickType joystick) {
        switch (joystick) {
            case LEFT:
                return -gamepad.left_stick_y; // Inverted to match standard coordinate system
            case RIGHT:
                return -gamepad.right_stick_y; // Inverted to match standard coordinate system
            default:
                return 0.0;
        }
    }

    /**
     * Checks if a joystick is pushed beyond the threshold in any direction.
     */
    public BooleanSupplier joystickMoved(JoystickType joystick) {
        return () -> {
            double x = 0;
            double y = 0;

            switch (joystick) {
                case LEFT:
                    x = gamepad.left_stick_x;
                    y = gamepad.left_stick_y;
                    break;
                case RIGHT:
                    x = gamepad.right_stick_x;
                    y = gamepad.right_stick_y;
                    break;
            }

            return Math.sqrt(x*x + y*y) > threshold;
        };
    }

    /**
     * Returns whether the gamepad is currently rumbling.
     */
    public boolean isRumbling() {
        return gamepad.isRumbling();
    }

    /**
     * Sets the gamepad to rumble for a specified duration.
     * @param durationMs The duration in milliseconds
     */
    public void rumble(int durationMs) {
        gamepad.rumble(durationMs);
    }

    /**
     * Sets the gamepad to rumble.
     *
     * @param leftPower Left rumble motor power (0.0 to 1.0)
     * @param rightPower Right rumble motor power (0.0 to 1.0)
     * @param durationMs The duration in milliseconds
     */
    public void rumble(double leftPower, double rightPower, int durationMs) {
        gamepad.rumble(leftPower, rightPower, durationMs);
    }

    /**
     * Enum for button types.
     */
    public enum ButtonType {
        A, B, X, Y,
        DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT,
        LEFT_BUMPER, RIGHT_BUMPER,
        LEFT_STICK_BUTTON, RIGHT_STICK_BUTTON,
        BACK, START, GUIDE
    }

    /**
     * Enum for trigger types.
     */
    public enum TriggerType {
        LEFT_TRIGGER, RIGHT_TRIGGER
    }

    /**
     * Enum for joystick types.
     */
    public enum JoystickType {
        LEFT, RIGHT
    }
}