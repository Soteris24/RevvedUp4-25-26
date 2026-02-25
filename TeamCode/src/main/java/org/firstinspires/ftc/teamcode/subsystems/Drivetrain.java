package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class Drivetrain {
    RobotHardware hw;
    Follower follower;
    PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    public boolean telemetryOn;
    //private final Pose startPose = new Pose(0, 0, Math.toRadians(45));

    public Drivetrain(RobotHardware hw, boolean telemetryOn) {
        this.hw = hw;
        this.telemetryOn = telemetryOn;
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
        hw.rightBack.setPower(rb);
    }

    public void stop() {
        setMotorPowers(0, 0, 0, 0);
    }

    public void facePoint(double targetX, double targetY) {
        Pose currentPose = follower.getPose();

        double dx = targetX - currentPose.getX();
        double dy = targetY - currentPose.getY();

        double targetHeading = Math.atan2(dy, dx);

        Pose targetPose = new Pose(
                currentPose.getX(),
                currentPose.getY(),
                targetHeading
        );

        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, targetPose))
                .setLinearHeadingInterpolation(
                        currentPose.getHeading(),
                        targetHeading
                )
                .build();

        follower.followPath(path);
    }
}