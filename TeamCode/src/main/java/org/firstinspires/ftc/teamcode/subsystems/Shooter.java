package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.Convertor;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.PIDFController;

@Configurable
public class Shooter {
    public RobotHardware hw;
    public PIDFController shooterPID;
    public PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    public boolean telemetryOn;

    public static double kP = 0.008; //0.05
    public static double kI = 0; //0.000067
    public static double kD = 0; //0.003
    public static double kV = 0.0002; // A guess

    public double targetVel = 0;
    public long lastTime = System.currentTimeMillis();

    public Shooter(RobotHardware hw, boolean telemetryOn) {
        this.hw = hw;
        this.shooterPID = new PIDFController(new PIDCoefficients(kP, kI, kD), kV);
        this.telemetryOn = telemetryOn;
    }

    public void update(){
        long currentTime = System.currentTimeMillis();
        double dt = Math.max((currentTime - lastTime) / 1000.0, 0.001);

        this.shooterPID.setCoefficients(new PIDCoefficients(kP,kI,kD), kV);
        double avgTPS = (hw.leftShooter.getVelocity() + hw.rightShooter.getVelocity()) / 2.0;
        double currentVel = Convertor.shooterTPStoRPM(avgTPS);

        double error = targetVel - currentVel;
        double power = shooterPID.update(error, dt, targetVel);

        this.setPower(power);

        if (this.telemetryOn) {
            panelsTelemetry.getTelemetry().addData("Shooter Vel", currentVel);
            panelsTelemetry.getTelemetry().addData("Target Vel", this.targetVel);
            panelsTelemetry.getTelemetry().addData("Shooter error", error);

            panelsTelemetry.getTelemetry().update();
        }

        this.lastTime = currentTime;

    }

    public void setPower(double power){
        this.hw.leftShooter.setPower(power);
        this.hw.rightShooter.setPower(power);
    }

    public void setTargetVel (double rpm)  {
        this.targetVel = rpm;
    }

    public void resetPID() {
        this.targetVel = 0;
        this.lastTime = System.currentTimeMillis();
        this.setPower(0);
        this.shooterPID.reset();
    }

    public boolean atTargetVel() {
        double avgTPS = (hw.leftShooter.getVelocity() + hw.rightShooter.getVelocity()) / 2.0;
        double currentVel = Convertor.shooterTPStoRPM(avgTPS);
        return (Math.abs(currentVel - targetVel) < 150);
    }

}
