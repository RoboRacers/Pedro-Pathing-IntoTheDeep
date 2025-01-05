package org.firstinspires.ftc.teamcode.teleop;

public class WeightedMovingAverage {
    private double currentAvg;
    private double weight;


    public WeightedMovingAverage(double weight) {
        this.weight = weight;
        this.currentAvg = 0;
    }



    public double getAvg(double value) {
        if (this.currentAvg == 0) {
            this.currentAvg=value;
        }
        else {
            this.currentAvg = this.currentAvg*(1-this.weight)+value*this.weight;
        }
        return this.currentAvg;
    }
}