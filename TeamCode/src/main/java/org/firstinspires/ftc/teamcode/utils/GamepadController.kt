package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.Gamepad
import java.util.function.BooleanSupplier
import java.util.function.Consumer
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * An enhanced controller for FTC Gamepad that provides event-based handling
 * for button presses and releases, along with other utility functions.
 */
class GamepadController
/**
 * Creates a new GamepadController for the given gamepad.
 *
 * @param gamepad The gamepad to monitor
 */(// The gamepad to monitor
    private val gamepad: Gamepad
) {
    // Maps to store the previous state of buttons
    private val prevButtonStates: MutableMap<BooleanSupplier, Boolean> = HashMap()

    // Maps to store callbacks for button events
    private val pressCallbacks: MutableMap<BooleanSupplier, Runnable> = HashMap()
    private val releaseCallbacks: MutableMap<BooleanSupplier, Runnable> = HashMap()
    private val whilePressedCallbacks: MutableMap<BooleanSupplier, Consumer<Double>> = HashMap()

    private var threshold = DEFAULT_THRESHOLD

    var lastOperationTime: Long = System.currentTimeMillis()
    /**
     * Sets the threshold for considering analog inputs as "pressed".
     *
     * @param threshold The threshold value (0.0 to 1.0)
     * @return This controller for method chaining
     */
    fun setThreshold(threshold: Double): GamepadController {
        this.threshold = max(0.0, min(1.0, threshold))
        return this
    }

    fun getNoOperationTime(): Long {
        return System.currentTimeMillis() - lastOperationTime
    }

    /**
     * Registers a callback to be called when a button is pressed.
     *
     * @param buttonSupplier The function that returns the button state
     * @param callback The callback to execute when the button is pressed
     * @return This controller for method chaining
     */
    fun onPressed(buttonSupplier: BooleanSupplier, callback: Runnable): GamepadController {
        pressCallbacks[buttonSupplier] = callback
        prevButtonStates.putIfAbsent(buttonSupplier, false)
        return this
    }

    /**
     * Registers a callback to be called when a button is pressed.
     * Simplified version that takes a ButtonType directly.
     *
     * @param button The button type to monitor
     * @param callback The callback to execute when the button is pressed
     * @return This controller for method chaining
     */
    fun onPressed(button: ButtonType, callback: Runnable): GamepadController {
        return onPressed(button(button), callback)
    }

    /**
     * Registers a callback to be called when a button is released.
     *
     * @param buttonSupplier The function that returns the button state
     * @param callback The callback to execute when the button is released
     * @return This controller for method chaining
     */
    fun onReleased(buttonSupplier: BooleanSupplier, callback: Runnable): GamepadController {
        releaseCallbacks[buttonSupplier] = callback
        prevButtonStates.putIfAbsent(buttonSupplier, false)
        return this
    }

    /**
     * Registers a callback to be called when a button is released.
     * Simplified version that takes a ButtonType directly.
     *
     * @param button The button type to monitor
     * @param callback The callback to execute when the button is released
     * @return This controller for method chaining
     */
    fun onReleased(button: ButtonType, callback: Runnable): GamepadController {
        return onReleased(button(button), callback)
    }

    /**
     * Registers a callback to be called continuously while a button is pressed.
     *
     * @param buttonSupplier The function that returns the button state
     * @param callback The callback to execute while the button is pressed,
     * receives a value from 0.0 to 1.0 for analog inputs
     * @return This controller for method chaining
     */
    fun whilePressed(
        buttonSupplier: BooleanSupplier,
        callback: Consumer<Double>
    ): GamepadController {
        whilePressedCallbacks[buttonSupplier] = callback
        prevButtonStates.putIfAbsent(buttonSupplier, false)
        return this
    }

    /**
     * Registers a callback to be called continuously while a button is pressed.
     * Simplified version that takes a ButtonType directly.
     *
     * @param button The button type to monitor
     * @param callback The callback to execute while the button is pressed
     * @return This controller for method chaining
     */
    fun whilePressed(button: ButtonType, callback: Consumer<Double>): GamepadController {
        return whilePressed(button(button), callback)
    }

    /**
     * Updates the controller state and executes any registered callbacks.
     * Should be called once per loop cycle.
     */
    fun update() {
        // Check button press/release events
        for (entry in prevButtonStates.entries) {
            val buttonSupplier = entry.key
            val prevState = entry.value
            val currentState = buttonSupplier.asBoolean

            // Check for button press, only when button pressed update lastOperationTime
            if (!prevState && currentState) {
                lastOperationTime = System.currentTimeMillis()
                val callback = pressCallbacks[buttonSupplier]
                callback?.run()
            }

            // Check for button release
            if (prevState && !currentState) {
                val callback = releaseCallbacks[buttonSupplier]
                callback?.run()
            }

            // Check for while pressed
            if (currentState) {
                val callback = whilePressedCallbacks[buttonSupplier]
                callback?.accept(1.0)
            }

            // Update previous state
            entry.setValue(currentState)
        }
    }

    /**
     * Creates a supplier for a gamepad button.
     */
    fun button(button: ButtonType): BooleanSupplier {
        return when (button) {
            ButtonType.A -> BooleanSupplier { gamepad.a }
            ButtonType.B -> BooleanSupplier { gamepad.b }
            ButtonType.X -> BooleanSupplier { gamepad.x }
            ButtonType.Y -> BooleanSupplier { gamepad.y }
            ButtonType.DPAD_UP -> BooleanSupplier { gamepad.dpad_up }
            ButtonType.DPAD_DOWN -> BooleanSupplier { gamepad.dpad_down }
            ButtonType.DPAD_LEFT -> BooleanSupplier { gamepad.dpad_left }
            ButtonType.DPAD_RIGHT -> BooleanSupplier { gamepad.dpad_right }
            ButtonType.LEFT_BUMPER -> BooleanSupplier { gamepad.left_bumper }
            ButtonType.RIGHT_BUMPER -> BooleanSupplier { gamepad.right_bumper }
            ButtonType.LEFT_STICK_BUTTON -> BooleanSupplier { gamepad.left_stick_button }
            ButtonType.RIGHT_STICK_BUTTON -> BooleanSupplier { gamepad.right_stick_button }
            ButtonType.BACK -> BooleanSupplier { gamepad.back }
            ButtonType.START -> BooleanSupplier { gamepad.start }
            ButtonType.GUIDE -> BooleanSupplier { gamepad.guide }
            else -> BooleanSupplier { false }
        }
    }

    /**
     * Creates a supplier for a gamepad trigger.
     */
    fun trigger(trigger: TriggerType): BooleanSupplier {
        return when (trigger) {
            TriggerType.LEFT_TRIGGER -> BooleanSupplier { gamepad.left_trigger > threshold }
            TriggerType.RIGHT_TRIGGER -> BooleanSupplier { gamepad.right_trigger > threshold }
            else -> BooleanSupplier { false }
        }
    }

    /**
     * Gets the raw value of a trigger (0.0 to 1.0).
     */
    fun getTriggerValue(trigger: TriggerType): Double {
        return when (trigger) {
            TriggerType.LEFT_TRIGGER -> gamepad.left_trigger.toDouble()
            TriggerType.RIGHT_TRIGGER -> gamepad.right_trigger.toDouble()
            else -> 0.0
        }
    }

    /**
     * Gets the X value of a joystick (-1.0 to 1.0).
     */
    fun getJoystickX(joystick: JoystickType): Double {
        return when (joystick) {
            JoystickType.LEFT -> gamepad.left_stick_x.toDouble()
            JoystickType.RIGHT -> gamepad.right_stick_x.toDouble()
            else -> 0.0
        }
    }

    /**
     * Gets the Y value of a joystick (-1.0 to 1.0).
     */
    fun getJoystickY(joystick: JoystickType): Double {
        return when (joystick) {
            JoystickType.LEFT -> -gamepad.left_stick_y.toDouble() // Inverted to match standard coordinate system
            JoystickType.RIGHT -> -gamepad.right_stick_y.toDouble() // Inverted to match standard coordinate system
            else -> 0.0
        }
    }

    /**
     * Checks if a joystick is pushed beyond the threshold in any direction.
     */
    fun joystickMoved(joystick: JoystickType): BooleanSupplier {
        return BooleanSupplier {
            var x = 0.0
            var y = 0.0

            when (joystick) {
                JoystickType.LEFT -> {
                    x = gamepad.left_stick_x.toDouble()
                    y = gamepad.left_stick_y.toDouble()
                }

                JoystickType.RIGHT -> {
                    x = gamepad.right_stick_x.toDouble()
                    y = gamepad.right_stick_y.toDouble()
                }
            }
            sqrt(x * x + y * y) > threshold
        }
    }

    val isRumbling: Boolean
        /**
         * Returns whether the gamepad is currently rumbling.
         */
        get() = gamepad.isRumbling

    /**
     * Sets the gamepad to rumble for a specified duration.
     * @param durationMs The duration in milliseconds
     */
    fun rumble(durationMs: Int) {
        gamepad.rumble(durationMs)
    }

    /**
     * Sets the gamepad to rumble.
     *
     * @param leftPower Left rumble motor power (0.0 to 1.0)
     * @param rightPower Right rumble motor power (0.0 to 1.0)
     * @param durationMs The duration in milliseconds
     */
    fun rumble(leftPower: Double, rightPower: Double, durationMs: Int) {
        gamepad.rumble(leftPower, rightPower, durationMs)
    }

    /**
     * Enum for button types.
     */
    enum class ButtonType {
        A, B, X, Y,
        DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT,
        LEFT_BUMPER, RIGHT_BUMPER,
        LEFT_STICK_BUTTON, RIGHT_STICK_BUTTON,
        BACK, START, GUIDE
    }

    /**
     * Enum for trigger types.
     */
    enum class TriggerType {
        LEFT_TRIGGER, RIGHT_TRIGGER
    }

    /**
     * Enum for joystick types.
     */
    enum class JoystickType {
        LEFT, RIGHT
    }

    companion object {
        // Threshold for considering analog inputs as "pressed"
        private const val DEFAULT_THRESHOLD = 0.1
    }
}