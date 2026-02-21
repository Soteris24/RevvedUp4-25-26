package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class PIDController {
    PIDCoefficients coefficients;

    double integral;
    double lastError;
    public PIDController(PIDCoefficients coefficients) {
        this.coefficients = coefficients;
    }

    public double update(double error, double dt) {
        double p = error * this.coefficients.p;

        integral += error * dt;
        double i =integral * this.coefficients.i;

        double derivative = (error - lastError) / dt;
        double d = derivative * this.coefficients.d;

        lastError = error;

        return p + i + d;
    }
    public void setCoefficients(PIDCoefficients coefficients) {
        this.coefficients = coefficients;
    }
    public  void reset(){
        lastError = 0;
        integral = 0;
    }
}
