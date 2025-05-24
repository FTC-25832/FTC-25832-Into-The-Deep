package org.firstinspires.ftc.teamcode.utils.gamepad;

public enum ButtonEnum {
    A,
    B,
    X,
    Y,
    LEFT_BUMPER,
    RIGHT_BUMPER,
    DPAD_UP,
    DPAD_DOWN,
    DPAD_LEFT,
    DPAD_RIGHT,
    LEFT_STICK_BUTTON, // Usually L3
    RIGHT_STICK_BUTTON, // Usually R3
    // For triggers, we'll consider them buttons if their value exceeds a threshold.
    // The actual thresholding logic will be handled in CustomGamepadEx or ButtonBinding.
    LEFT_TRIGGER,
    RIGHT_TRIGGER,
    START, // Or Options button
    BACK // Or Share button
}
