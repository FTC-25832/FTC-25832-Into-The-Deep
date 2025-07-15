package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class GamepadController {
    private static final double DEFAULT_THRESHOLD = 0.1;
    private final Gamepad gamepad;
    private final Map<BooleanSupplier, Boolean> prevButtonStates = new HashMap<>();
    private final Map<BooleanSupplier, Runnable> pressCallbacks = new HashMap<>();
    private final Map<BooleanSupplier, Runnable> releaseCallbacks = new HashMap<>();
    private final Map<BooleanSupplier, Consumer<Double>> whilePressedCallbacks = new HashMap<>();
    private double threshold = DEFAULT_THRESHOLD;
    private long lastOperationTime = System.currentTimeMillis();

    public GamepadController(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public GamepadController setThreshold(double threshold) {
        this.threshold = Math.max(0.0, Math.min(1.0, threshold));
        return this;
    }

    public long getNoOperationTime() {
        return System.currentTimeMillis() - lastOperationTime;
    }

    public GamepadController onPressed(BooleanSupplier buttonSupplier, Runnable callback) {
        pressCallbacks.put(buttonSupplier, callback);
        prevButtonStates.putIfAbsent(buttonSupplier, false);
        return this;
    }

    public GamepadController onPressed(ButtonType button, Runnable callback) {
        return onPressed(button(button), callback);
    }

    public GamepadController onReleased(BooleanSupplier buttonSupplier, Runnable callback) {
        releaseCallbacks.put(buttonSupplier, callback);
        prevButtonStates.putIfAbsent(buttonSupplier, false);
        return this;
    }

    public GamepadController onReleased(ButtonType button, Runnable callback) {
        return onReleased(button(button), callback);
    }

    public GamepadController whilePressed(BooleanSupplier buttonSupplier, Consumer<Double> callback) {
        whilePressedCallbacks.put(buttonSupplier, callback);
        prevButtonStates.putIfAbsent(buttonSupplier, false);
        return this;
    }

    public GamepadController whilePressed(ButtonType button, Consumer<Double> callback) {
        return whilePressed(button(button), callback);
    }

    public void update() {
        for (Map.Entry<BooleanSupplier, Boolean> entry : prevButtonStates.entrySet()) {
            BooleanSupplier buttonSupplier = entry.getKey();
            boolean prevState = entry.getValue();
            boolean currentState = buttonSupplier.getAsBoolean();

            if (!prevState && currentState) {
                lastOperationTime = System.currentTimeMillis();
                Runnable callback = pressCallbacks.get(buttonSupplier);
                if (callback != null) {
                    callback.run();
                }
            }

            if (prevState && !currentState) {
                Runnable callback = releaseCallbacks.get(buttonSupplier);
                if (callback != null) {
                    callback.run();
                }
            }

            if (currentState) {
                Consumer<Double> callback = whilePressedCallbacks.get(buttonSupplier);
                if (callback != null) {
                    callback.accept(1.0);
                }
            }

            entry.setValue(currentState);
        }
    }

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

    public BooleanSupplier joystickMoved(JoystickType joystick) {
        return () -> {
            double x = 0.0;
            double y = 0.0;

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
            return Math.sqrt(x * x + y * y) > threshold;
        };
    }

    public boolean isRumbling() {
        return gamepad.isRumbling();
    }

    public void rumble(int durationMs) {
        gamepad.rumble(durationMs);
    }

    public void rumble(double leftPower, double rightPower, int durationMs) {
        gamepad.rumble(leftPower, rightPower, durationMs);
    }

    public enum ButtonType {
        A, B, X, Y,
        DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT,
        LEFT_BUMPER, RIGHT_BUMPER,
        LEFT_STICK_BUTTON, RIGHT_STICK_BUTTON,
        BACK, START, GUIDE
    }

    public enum TriggerType {
        LEFT_TRIGGER, RIGHT_TRIGGER
    }

    public enum JoystickType {
        LEFT, RIGHT
    }
}