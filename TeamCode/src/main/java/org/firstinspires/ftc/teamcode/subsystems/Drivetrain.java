package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class Drivetrain {
    RobotHardware hw;
    PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    public boolean telemetryOn;

    public Drivetrain(RobotHardware hw, boolean telemetryOn) {
        this.hw = hw;
        this.telemetryOn = telemetryOn;
    }
    public void drive(double y, double x, double rx) {
        double angle = Math.atan2(y,x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(angle + Math.PI/4);
        double cos = -Math.cos(angle + Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double lf = (sin * power) / max + rx;
        double lb = (cos * power) / max + rx;
        double rf = (cos * power) / max - rx;
        double rb = (sin * power) / max - rx;

        double maxPower = Math.max(Math.abs(lf), Math.max(Math.abs(lb), Math.max(Math.abs(rf), Math.abs(rb))));
        if (maxPower > 1.0) {
            lf /= maxPower;
            lb /= maxPower;
            rf /= maxPower;
            rb /= maxPower;
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
        hw.rightBack.setPower(rb);
    }

    public void stop() {
        setMotorPowers(0, 0, 0, 0);
    }

}

