package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class Intake {
    public RobotHardware hw;
    public PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    public boolean telemetryOn;

    public boolean intakeOn = false;
    public boolean lastButton = false;
    public boolean reversing = false;
    public  double reverseTime = 0;

    public double forwardPower = 1;
    public double reversePower = -1;
    public double reverseDuration = 0.5;


    public Intake(RobotHardware hw, boolean telemetryOn) {
        this.hw = hw;
        this.telemetryOn = telemetryOn;
    }

    public void intake(boolean button, boolean reverseButton, double currentTime){
        if (button && !lastButton) {
            intakeOn = !intakeOn;
        }

        if (reverseButton) {
            reversing = true;
            reverseTime = currentTime + reverseDuration;
        }
        if (reversing) {
            hw.intake.setPower(reversePower);
            if (!reverseButton && currentTime >= reverseTime) {
                reversing = false;
            }
        } else {
            hw.intake.setPower(intakeOn ? forwardPower : 0);
        }
        if (this.telemetryOn) {
            panelsTelemetry.getTelemetry().addData("Intake On", intakeOn);
        }

        lastButton = button;
    }

    public void stop() {
        hw.intake.setPower(0);
        intakeOn = false;
        reversing = false;
        reverseTime = 0;
    }
}
