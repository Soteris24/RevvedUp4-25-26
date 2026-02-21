package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class PIDFController {
    PIDCoefficients coefficients;
    double kV;

    double integral;
    double lastError;
    public PIDFController(PIDCoefficients coefficients, double kV) {
        this.coefficients = coefficients;
        this.kV = kV;
    }

    public double update(double error, double dt, double targetVelocity) {
        double p = error * this.coefficients.p;

        integral += error * dt;
        double i =integral * this.coefficients.i;

        double derivative = (error - lastError) / dt;
        double d = derivative * this.coefficients.d;

        lastError = error;

        double ff = kV * targetVelocity;

        return p + i + d;
    }
    public void setCoefficients(PIDCoefficients coefficients, double kV) {
        this.coefficients = coefficients;
        this.kV = kV;
    }

    public  void reset(){
        lastError = 0;
        integral = 0;
    }
}
