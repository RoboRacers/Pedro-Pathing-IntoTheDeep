package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double kP, kI, kD;
    private double setpoint;
    private double lastError;
    private double integral;
    private double outputMin = -1000;
    private double outputMax = 1000;
    private double errorTolerance = 0; // Default error range
    private ElapsedTime timer = new ElapsedTime();

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.lastError = 0;
        this.integral = 0;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void setOutputLimits(double min, double max) {
        outputMin = min;
        outputMax = max;
    }

    public double getSetpoint(){
        return setpoint;
    }

    public void setCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setErrorTolerance(double tolerance) {
        this.errorTolerance = tolerance;
    }

    public double calculate(double current) {

        double deltaTime = timer.seconds();
        timer.reset();

        // Calculate error
        double error = setpoint - current;

        // If the error is within the tolerance range, output zero
        if (Math.abs(error) <= errorTolerance) {
            reset(); // Optionally reset integral and lastError
            return 0;
        }

        // Proportional term
        double P = kP * error;

        // Integral term
        integral += error * deltaTime;
        double I = kI * integral;

        // Derivative term
        double D = kD * (error - lastError) / deltaTime;

        // Compute the output
        double output = P + I + D;

        // Save the current error as the last error for the next cycle
        lastError = error;

        // Clamp output to min and max bounds
        output = Math.max(outputMin, Math.min(outputMax, output));

        return output;
    }

    public void reset() {
        lastError = 0;
        integral = 0;
    }
}