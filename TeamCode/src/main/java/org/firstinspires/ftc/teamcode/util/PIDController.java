package org.firstinspires.ftc.teamcode.util;
/**
 * A Proportional-Integral-Derivative (PID) controller implementation.
 * Example usage:
 * public PIDController pidController;
 * initialize() {
 *      pidController = new PIDController(PIDConstant.Kp, PIDConstant.Ki, PIDConstant.Kd);
 * }
 * pidController.setDestination(dest);
 * loop{
 *      double power = pidController.calculate(currentPosition);
 * }
 * 
 */
public class PIDController {
    // Controller gains
    public double kp; // Proportional gain
    public double ki; // Integral gain
    public double kd; // Derivative gain

    // Controller state
    public double destination; // Desired value
    public double integralSum; // Sum of error over time
    public double lastError; // Previous error
    public long lastTime; // Last execution time in milliseconds
    public boolean isInitialized; // Flag to check if controller has been initialized

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.destination = 0;
        this.integralSum = 0;
        this.lastError = 0;
        this.lastTime = 0;
        this.isInitialized = false;
    }

    /**
     * Set the desired destination for the controller.
     */
    public void setDestination(double destination) {
        this.destination = destination;
        reset();
    }

    public void setKp(double kp) {
        this.kp = kp;
    }

    public void setKi(double ki) {
        this.ki = ki;
    }

    public void setKd(double kd) {
        this.kd = kd;
    }

    /**
     * Reset the controller state.
     */
    public void reset() {
        this.integralSum = 0;
        this.lastError = 0;
        this.isInitialized = false;
    }

    /**
     * Calculate the control output based on the measured process value.
     */
    public double calculate(double processValue) {
        long currentTime = System.currentTimeMillis();

        // Initialize controller on first call
        if (!isInitialized) {
            isInitialized = true;
            lastTime = currentTime;
            lastError = destination - processValue;
            return kp * lastError;
        }

        double deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
        lastTime = currentTime;

        // Calculate error
        double error = destination - processValue;

        // Proportional term
        double proportional = kp * error;

        // Integral term
        if (deltaTime > 0) {
            integralSum += ki * error * deltaTime;
        }

        // Derivative term
        double derivative = 0;
        if (deltaTime > 0) {
            derivative = kd * (error - lastError) / deltaTime;
        }
        lastError = error;

        // Calculate output without limits
        double result = proportional + integralSum + derivative;
        return Math.min(Math.max(result, -1), 1); // Limit output to [-1, 1]
    }
}