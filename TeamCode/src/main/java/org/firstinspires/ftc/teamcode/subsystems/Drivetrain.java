package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.PIDController;

public class Drivetrain {
    RobotHardware hw;
    Follower follower;
    PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    public boolean telemetryOn;

    public PIDController headingPID;
    public PIDController odometryHeadingPID;
    private ElapsedTime pidTimer = new ElapsedTime();
    public static PIDCoefficients headingCoeffs = new PIDCoefficients(2, 0, 0.21);
    public static PIDCoefficients odometryCoeffs = new PIDCoefficients(0.2, 0, 0.0);

    public Drivetrain(RobotHardware hw, Follower follower, boolean telemetryOn) {
        this.hw = hw;
        this.follower = follower;
        this.telemetryOn = telemetryOn;
        this.headingPID = new PIDController(headingCoeffs);
        this.odometryHeadingPID = new PIDController(odometryCoeffs);
    }

    public void drive(double y, double x, double rx) {
        double lf = y + x + rx;
        double lb = y - x + rx;
        double rf = y - x - rx;
        double rb = y + x - rx;

        double max = Math.max(Math.abs(lf), Math.max(Math.abs(lb), Math.max(Math.abs(rf), Math.abs(rb))));
        if (max > 1.0) {
            lf /= max;
            lb /= max;
            rf /= max;
            rb /= max;
        }

        hw.leftFront.setPower(lf);
        hw.leftBack.setPower(lb);
        hw.rightFront.setPower(rf);
        hw.rightBack.setPower(rb);

        if (telemetryOn) {
            panelsTelemetry.getTelemetry().addData("LF", lf);
            panelsTelemetry.getTelemetry().addData("LB", lb);
            panelsTelemetry.getTelemetry().addData("RF", rf);
            panelsTelemetry.getTelemetry().addData("RB", rb);
        }
    }

    public void setMotorPowers(double lf, double lb, double rf, double rb) {
        hw.leftFront.setPower(lf);
        hw.leftBack.setPower(lb);
        hw.rightFront.setPower(rf);
        hw.rightBack.setPower(-rb);
    }

    public void stop() {
        setMotorPowers(0, 0, 0, 0);
    }

    private double lastKnownHeading = Double.NaN;

    public double faceLimelightTarget() {
        LLResult result = hw.limelight.getLatestResult();
        
        // 1. If camera has a valid target
        if (result != null && result.isValid()) {
            double tx = result.getTx(); 
            
            double dt = pidTimer.seconds();
            pidTimer.reset();

            if (dt > 0.5) {
                headingPID.reset();
                dt = 0.01;
            }

            // Only update "memory" when we are actually "on target" (e.g., < 1 degree)
            if (Math.abs(tx) < 1.0) {
                lastKnownHeading = follower.getPose().getHeading() + Math.toRadians(tx);
            }

            double output = headingPID.update(Math.toRadians(tx), dt);
            return Math.max(-1.0, Math.min(1.0, output));
        }
        
        // 2. If no target, use saved memory if available
        if (!Double.isNaN(lastKnownHeading)) {
            double currentHeading = follower.getPose().getHeading() + Math.toRadians(180);
            double headingError = lastKnownHeading - currentHeading;
            
            // Wrap error
            while (headingError > Math.PI)  headingError -= 2 * Math.PI;
            while (headingError < -Math.PI) headingError += 2 * Math.PI;
            
            double dt = pidTimer.seconds();
            pidTimer.reset();
            if (dt > 0.5) { odometryHeadingPID.reset(); dt = 0.01; }

            double output = odometryHeadingPID.update(headingError, dt);
            return Math.max(-1.0, Math.min(1.0, output));
        }

        return 0; // No target, no memory
    }

    public double facePoint(double targetX, double targetY) {
        Pose currentPose = follower.getPose();

        double dx = targetX - currentPose.getX();
        double dy = targetY - currentPose.getY();

        double targetHeading = Math.atan2(dy, dx);
        double currentHeading = currentPose.getHeading() + Math.toRadians(180);

        double headingError = targetHeading - currentHeading;
        while (headingError > Math.PI)  headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;

        double dt = pidTimer.seconds();
        pidTimer.reset();

        if (dt > 0.5) {
            headingPID.reset();
            dt = 0.01;
        }

        double output = headingPID.update(headingError, dt);
        return Math.max(-1.0, Math.min(1.0, output));
    }
}
