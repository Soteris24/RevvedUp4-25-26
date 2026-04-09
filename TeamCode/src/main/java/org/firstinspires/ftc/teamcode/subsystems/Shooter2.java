package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.Convertor;

@Configurable
public class Shooter2 {
RobotHardware hw;

public static double kP = 0.02;
public static double kI = 0;
public static double kD = 0.02;
public static double kV = 10.1;

public double targetVelRPM;
public double targetVelTPS;

public PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
public  boolean telemetryOn;
public double atTargetStartTime = -1;
public double atTargetDelayTime = 600;

    public Shooter2 (RobotHardware hw, boolean telemetryOn) {
        this.hw = hw;
        this.telemetryOn = telemetryOn;
   }

    public void setPIDFCoefficients() {
        double voltage = hw.getBatteryVoltage();

        double compensatedF = kV * (13.0 / voltage);

        hw.leftShooter.setVelocityPIDFCoefficients(kP, kI, kD, compensatedF);
        hw.rightShooter.setVelocityPIDFCoefficients(kP, kI, kD, compensatedF);
    }

   public void setTargetVelRPM(double RPM){
        targetVelRPM = RPM;
        targetVelTPS = Convertor.shooterTPStoRPM(targetVelRPM);
        hw.leftShooter.setVelocity(targetVelRPM);
        hw.rightShooter.setVelocity(targetVelRPM);
        updateTelemetry();
   }
   public double currentVelocity(){
        double leftShooterRPM = Convertor.shooterTPStoRPM(hw.leftShooter.getVelocity(AngleUnit.DEGREES));
        double rightShooterRPM = Convertor.shooterTPStoRPM(hw.rightShooter.getVelocity(AngleUnit.DEGREES));
        return  (rightShooterRPM + rightShooterRPM) / 2;
    }
    public boolean atTargetVel(){
        double error = targetVelRPM - currentVelocity();
        boolean withinThreshold = Math.abs(error) < 300;

        long now = System.currentTimeMillis();
        if (withinThreshold) {
            if (atTargetStartTime == -1) {
                atTargetStartTime = now;
            }
            return (now - atTargetStartTime >= atTargetDelayTime);
        }
        else {
            atTargetStartTime = -1;
            return false;
        }
    }
    // Distance in inches → RPM lookup table (add more points as you test!)
    private static final double[] DISTANCE_TABLE = { 24,    48,    72,    96  };
    private static final double[] RPM_TABLE      = { 2100,  2250,  2450,  2700};

    public void setVelocityForDistance(double distanceInches) {
        double rpm = interpolateRPM(distanceInches);
        setTargetVelRPM(rpm);
        if (telemetryOn) {
            panelsTelemetry.getTelemetry().addData("Auto RPM", rpm);
            panelsTelemetry.getTelemetry().addData("Distance", distanceInches);
        }
    }

    private double interpolateRPM(double distance) {
        // Below minimum — clamp to first entry
        if (distance <= DISTANCE_TABLE[0]) return RPM_TABLE[0];
        // Above maximum — clamp to last entry
        if (distance >= DISTANCE_TABLE[DISTANCE_TABLE.length - 1]) return RPM_TABLE[RPM_TABLE.length - 1];

        // Find which two points we're between and lerp
        for (int i = 0; i < DISTANCE_TABLE.length - 1; i++) {
            if (distance >= DISTANCE_TABLE[i] && distance <= DISTANCE_TABLE[i + 1]) {
                double t = (distance - DISTANCE_TABLE[i]) / (DISTANCE_TABLE[i + 1] - DISTANCE_TABLE[i]);
                return RPM_TABLE[i] + t * (RPM_TABLE[i + 1] - RPM_TABLE[i]);
            }
        }
        return RPM_TABLE[RPM_TABLE.length - 1];
    }

    public void updateTelemetry(){
//        panelsTelemetry.getTelemetry().addData("left shooter", Convertor.shooterTPStoRPM(hw.leftShooter.getVelocity(AngleUnit.DEGREES)));
//        panelsTelemetry.getTelemetry().addData("right shooter", Convertor.shooterTPStoRPM(hw.rightShooter.getVelocity(AngleUnit.DEGREES)));
//        panelsTelemetry.getTelemetry().addData("shooter target", targetVelRPM);
//        panelsTelemetry.getTelemetry().addData("shooter vel", currentVelocity());
//        panelsTelemetry.getTelemetry().update();
    }
}
