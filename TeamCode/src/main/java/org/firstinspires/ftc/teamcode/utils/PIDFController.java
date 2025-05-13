package org.firstinspires.ftc.teamcode.utils;

/**
 * A Proportional-Integral-Derivative-Feedforward (PIDF) controller
 * implementation.
 * Example usage:
 * public PIDFController pidfController;
 * initialize() {
 * pidfController = new PIDFController(PIDConstant.Kp, PIDConstant.Ki,
 * PIDConstant.Kd, PIDConstant.Kf);
 * }
 * pidfController.setDestination(dest);
 * loop{
 * double power = pidfController.calculate(currentPosition);
 * }
 * 
 */
public class PIDFController {
    // Controller gains
    public double kp; // Proportional gain
    public double ki; // Integral gain
    public double kd; // Derivative gain
    public double kf; // Feedforward gain

    // Controller state
    public double pos;
    public double destination; // Desired value
    public double integralSum; // Sum of error over time
    public double lastError; // Previous error
    public long lastTime; // Last execution time in milliseconds
    public boolean isInitialized; // Flag to check if controller has been initialized

    public PIDFController(double kp, double ki, double kd, double kf) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
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

    public void setKf(double kf) {
        this.kf = kf;
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
        pos = processValue;

        // Initialize controller on first call
        if (!isInitialized) {
            isInitialized = true;
            lastTime = currentTime;
            lastError = destination - processValue;
            // Initial output includes feedforward
            return kp * lastError + kf * destination;
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

        // Feedforward term
        double feedforward = kf * destination;

        double result = proportional + integralSum + derivative + feedforward;
        return Math.min(Math.max(result, -1), 1); // Limit output to [-1, 1]
    }
}
