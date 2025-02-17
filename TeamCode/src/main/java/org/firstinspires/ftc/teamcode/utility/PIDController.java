package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.util.ElapsedTime;

public final class PIDController {
    private double Kp, Ki, Kd, KStaticPositive, KStaticNegative;
    private double tolerance;
    private final ElapsedTime timer;

    private double lastError, integralSum;
    private boolean isFirstIteration;

    public PIDController(double Kp, double Ki, double Kd, double KStaticPositive, double KStaticNegative) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.KStaticPositive = KStaticPositive;
        this.KStaticNegative = KStaticNegative;
        this.tolerance = 0.0;
        timer = new ElapsedTime();
        reset();
    }

    public double calculate(double currentPosition, double targetPosition) {
        double error = targetPosition - currentPosition;
        double output;

        if (isFirstIteration) {
            output = Kp * error;
            if (error < 0) {
                output += KStaticNegative;
            } else if (error > 0) {
                output += KStaticPositive;
            }
            lastError = error;
            timer.reset();
            isFirstIteration = false;
        } else {
            // Rate of change of the error
            double deltaTime = timer.seconds();
            double derivative = deltaTime != 0.0 ? (error - lastError) / deltaTime : 0.0;

            // Sum of all error over time
            integralSum = integralSum + error * deltaTime;

            output = Kp * error + Ki * integralSum + Kd * derivative;

            if (error > 0.0) {
                output += KStaticPositive;
            } else if (error < 0.0) {
                output += KStaticNegative;
            }

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

    public void debugSetCoefficients(
            double P,
            double I,
            double D,
            double STATIC_POSITIVE,
            double STATIC_NEGATIVE
    ) {
        this.Kp = P;
        this.Ki = I;
        this.Kd = D;
        this.KStaticPositive = STATIC_POSITIVE;
        this.KStaticNegative = STATIC_NEGATIVE;
    }

    public void reset() {
        lastError        = 0.0;
        integralSum      = 0.0;
        isFirstIteration = true;
    }
}