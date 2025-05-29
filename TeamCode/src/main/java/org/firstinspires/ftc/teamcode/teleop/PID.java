package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class PID {
    private double kP;
    private double kI;
    private double kD;
    private Timer time;
    private double target;
    private double previousError;

    public PID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        previousError = 0;
    }

    public double calculate(double currentPosition) {
        double elapsedTime = time.getElapsedTime();
        double error = target - currentPosition;

        double proportional = kP * error;
        double integral = kI * (error + previousError) * elapsedTime / 2;
        double derivative = kD * (error - previousError) / elapsedTime;

        time.resetTimer();
        previousError = error;
        return proportional + integral + derivative;
    }

    public void setTarget(double target) {this.target = target;}

    public double getTarget() {return target;}

    public void setKP(double kP) {this.kP = kP;}

    public void setKI(double kI) {this.kI = kI;}

    public void setKD(double kD) {this.kD = kD;}

    public double getKP() {return kP;}

    public double getKI() {return kI;}

    public double getKD() {return kD;}
}
