package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.util.ElapsedTime;

public final class PIDController {
    private double kp, ki, kd;
    private double tolerance;
    private final ElapsedTime timer;

    private double lastError, integralSum;
    private boolean isFirstIteration;

    public PIDController(double Kp, double Ki, double Kd) {
        this.kp = Kp;
        this.ki = Ki;
        this.kd = Kd;
        this.tolerance = 0.0;
        timer = new ElapsedTime();
        reset();
    }

    public double calculate(double currentPosition, double targetPosition) {
        double error = targetPosition - currentPosition;
        double output;

        if (isFirstIteration) {
            output = kp * error;
            lastError = error;
            timer.reset();
            isFirstIteration = false;
        } else {
            // Rate of change of the error
            double deltaTime = timer.seconds();
            double derivative = deltaTime != 0.0 ? (error - lastError) / deltaTime : 0.0;

            // Sum of all error over time
            integralSum = integralSum + error * deltaTime;

            output = (kp * error) + (ki * integralSum) + (kd * derivative);

            lastError = error;

            // Reset the timer for the next time
            timer.reset();
        }

        if (Math.abs(error) < tolerance) output = 0.0;

        return output;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = Math.max(0, tolerance);
    }

    public void setCoefficients(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public void reset() {
        lastError        = 0.0;
        integralSum      = 0.0;
        isFirstIteration = true;
    }
}