package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Configurable
public class Drivetrain {
    public static double GOAL_X_IN = 40.0;
    public static double GOAL_Y_IN = 0.0;

    public static double SHOOTER_OFFSET_X_IN = 0.0;
    public static double SHOOTER_OFFSET_Y_IN = 0.0;
    public static double SHOOTER_FORWARD_OFFSET_DEG = 180.0;

    public static double AIM_KP = 0.95;
    public static double AIM_KD = 0.10;
    public static double AIM_MIN_OUTPUT = 0.06;
    public static double AIM_MAX_OUTPUT = 0.75;
    public static double AIM_DEADBAND_DEG = 1.0;

    RobotHardware hw;
    Follower follower;
    PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    ElapsedTime aimTimer = new ElapsedTime();
    public boolean telemetryOn;

    private double lastHeadingError;
    private boolean aimControllerPrimed;

    public Drivetrain(RobotHardware hw, Follower follower, boolean telemetryOn) {
        this.hw = hw;
        this.follower = follower;
        this.telemetryOn = telemetryOn;
        aimTimer.reset();
    }

    public void drive(double y, double x, double rx) {
        // =========================
        // DRIVE
        // =========================
        double lf = y + x + rx;
        double lb = y - x + rx;
        double rf = y - x - rx;
        double rb = y + x - rx;

        // Cap powers to [-1, 1]
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

        /* =========================
        // ORIGINAL DRIVE CODE

        double angle = Math.atan2(y,x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(angle + Math.PI/4);
        double cos = -Math.cos(angle + Math.PI/4);
        double maxOrig = Math.max(Math.abs(sin), Math.abs(cos));

        double lfOrig = (sin * power) / maxOrig + rx;
        double lbOrig = (cos * power) / maxOrig + rx;
        double rfOrig = (cos * power) / maxOrig - rx;
        double rbOrig = (sin * power) / maxOrig - rx;

        double maxPower = Math.max(Math.abs(lfOrig), Math.max(Math.abs(lbOrig), Math.max(Math.abs(rfOrig), Math.abs(rbOrig))));
        if (maxPower > 1.0) {
            lfOrig /= maxPower;
            lbOrig /= maxPower;
            rfOrig /= maxPower;
            rbOrig /= maxPower;
        }

        hw.leftFront.setPower(lfOrig);
        hw.leftBack.setPower(lbOrig);
        hw.rightFront.setPower(rfOrig);
        hw.rightBack.setPower(rbOrig);
        */
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

    public double faceGoal() {
        return facePoint(GOAL_X_IN, GOAL_Y_IN);
    }

    public double facePoint(double targetX, double targetY) {
        Pose currentPose = follower.getPose();
        Pose shooterPose = getShooterPose(currentPose);

        double dx = targetX - shooterPose.getX();
        double dy = targetY - shooterPose.getY();

        double targetHeading = Math.atan2(dy, dx);
        double currentHeading = shooterPose.getHeading();

        double headingError = normalizeRadians(targetHeading - currentHeading);
        double dt = aimTimer.seconds();
        aimTimer.reset();

        double derivative = 0.0;
        if (aimControllerPrimed && dt > 1e-4) {
            derivative = (headingError - lastHeadingError) / dt;
        }

        lastHeadingError = headingError;
        aimControllerPrimed = true;

        double headingErrorDeg = Math.toDegrees(headingError);
        if (Math.abs(headingErrorDeg) <= AIM_DEADBAND_DEG) {
            return 0.0;
        }

        double output = AIM_KP * headingError - AIM_KD * derivative;
        double minOutput = Math.copySign(AIM_MIN_OUTPUT, output);
        if (Math.abs(output) < AIM_MIN_OUTPUT) {
            output = minOutput;
        }

        output = Math.max(-AIM_MAX_OUTPUT, Math.min(AIM_MAX_OUTPUT, output));

        if (telemetryOn) {
            panelsTelemetry.getTelemetry().addData("Aim Error Deg", headingErrorDeg);
            panelsTelemetry.getTelemetry().addData("Aim Target Deg", Math.toDegrees(targetHeading));
            panelsTelemetry.getTelemetry().addData("Aim Output", output);
            panelsTelemetry.getTelemetry().addData("Shooter X", shooterPose.getX());
            panelsTelemetry.getTelemetry().addData("Shooter Y", shooterPose.getY());
        }

        return output;
    }

    public void resetAimController() {
        aimControllerPrimed = false;
        lastHeadingError = 0.0;
        aimTimer.reset();
    }

    public boolean isAimedAtGoal() {
        Pose shooterPose = getShooterPose(follower.getPose());
        double targetHeading = Math.atan2(GOAL_Y_IN - shooterPose.getY(), GOAL_X_IN - shooterPose.getX());
        double headingError = normalizeRadians(targetHeading - shooterPose.getHeading());
        return Math.abs(Math.toDegrees(headingError)) <= AIM_DEADBAND_DEG;
    }

    private Pose getShooterPose(Pose robotPose) {
        double heading = robotPose.getHeading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double shooterX = robotPose.getX() + SHOOTER_OFFSET_X_IN * cos - SHOOTER_OFFSET_Y_IN * sin;
        double shooterY = robotPose.getY() + SHOOTER_OFFSET_X_IN * sin + SHOOTER_OFFSET_Y_IN * cos;
        double shooterHeading = normalizeRadians(heading + Math.toRadians(SHOOTER_FORWARD_OFFSET_DEG));

        return new Pose(shooterX, shooterY, shooterHeading);
    }

    private double normalizeRadians(double angle) {
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }
}
