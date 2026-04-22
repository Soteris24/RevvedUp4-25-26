package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.Convertor;

@Configurable
public class Sorter {

    public RobotHardware hw;
    public PIDController sorterPID;
    //RobotHardware hw;
    private final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    public boolean telemetryOn;

    // ===== PID TUNING =====
    public static double kP = 0.0005; //0.0003
    public static double kI = 0.0;
    public static double kD = 0.000018; //0.000004

    // ===== CONTROL PARAMS =====
    public static double minPower = 0.1;   // overcome static friction
    public static int slowZone = 700;        // ticks
    public static int targetTolerance = 200;  // ticks
    public static long settleTime = 50;      // ms

    // ===== STUCK DETECTION PARAMS =====
    public static double stuckErrorThreshold = 150;  // error must be within this to not count as "stuck"
    public static long stuckDetectionTime = 1000;    // ms - how long error must be stable to trigger reverse
    public static double reverseDistance = 10;       // degrees to reverse
    public static long reverseTime = 30;            // ms - how long to reverse
    public static long reverseRecoveryTime = 150;    // ms - wait before retrying after reverse

    public int targetTicks;
    private long lastTime;

    private long stableSince = 0;

    // ===== STUCK DETECTION STATE =====
    private double lastError = 0;
    private long lastErrorChangeTime = 0;
    private int stuckDetectionState = 0;  // 0: normal, 1: reversing, 2: recovering
    private long stuckStateStartTime = 0;

    // ===== TRANSFER STATE =====
    private int transferState = 0;
    private double transferStartTime = 0;

    public Sorter(RobotHardware hw, boolean telemetryOn) {
        this.hw = hw;
        this.telemetryOn = telemetryOn;

        hw.sorter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sorterPID = new PIDController(new PIDCoefficients(kP , kI, kD)); //* (12.8 / hw.getBatteryVoltage())


        targetTicks = hw.sorter.getCurrentPosition();
        lastTime = System.currentTimeMillis();
        lastErrorChangeTime = System.currentTimeMillis();
    }

    // ================= UPDATE LOOP =================
    public void update() {
        long now = System.currentTimeMillis();
        double dt = Math.max((now - lastTime) / 1000.0, 0.001);
        lastTime = now;

        sorterPID.setCoefficients(new PIDCoefficients(kP , kI, kD));

        int position = hw.sorter.getCurrentPosition();
        double error = targetTicks - position;

        // ===== STUCK DETECTION & REVERSAL =====
        handleStuckDetection(error, now);

        double power;

        // If we're in a stuck recovery state, handle reversal logic
        if (stuckDetectionState == 1) {
            // Reversing state - apply reverse power
            power = -0.3; // Reverse at moderate power
            hw.sorter.setPower(power);
        } else if (stuckDetectionState == 2) {
            // Recovery state - hold position, let it settle
            power = 0;
            hw.sorter.setPower(power);
        } else {
            // Normal PID control
            power = sorterPID.update(error, dt);

            // Slow down near target
            double scale = Math.min(1.0, Math.abs(error) / slowZone);
            power *= scale;

            // Minimum power clamp (prevents stalling & wiggle)
            if (Math.abs(power) < minPower && Math.abs(error) > targetTolerance) {
                power = Math.signum(power) * minPower;
            }

            power = Math.max(-1.0, Math.min(1.0, power));
            hw.sorter.setPower(power);
        }

        if (telemetryOn) {
            panelsTelemetry.getTelemetry().addData("Sorter Pos", position);
            panelsTelemetry.getTelemetry().addData("Target", targetTicks);
            panelsTelemetry.getTelemetry().addData("Error", error);
            panelsTelemetry.getTelemetry().addData("Power", power);
            panelsTelemetry.getTelemetry().addData("Stuck State", stuckDetectionState);
        }
    }

    // ================= STUCK DETECTION =================
    private void handleStuckDetection(double error, long now) {
        // Check if error has changed significantly
        if (Math.abs(error - lastError) > stuckErrorThreshold) {
            // Error changed, reset timer
            lastErrorChangeTime = now;
            lastError = error;
        }

        long stuckDuration = now - lastErrorChangeTime;

        switch (stuckDetectionState) {
            case 0: // Normal operation
                // If error is stable for 2 seconds and not at target, trigger reversal
                if (stuckDuration > stuckDetectionTime &&
                        Math.abs(error) > targetTolerance) {
                    stuckDetectionState = 1;
                    stuckStateStartTime = now;
                }
                break;

            case 1: // Reversing
                if (now - stuckStateStartTime > reverseTime) {
                    // Switch to recovery state
                    stuckDetectionState = 2;
                    stuckStateStartTime = now;
                    // Adjust target slightly backward to give it room
                    targetTicks -= Convertor.toTicks(reverseDistance);
                    sorterPID.reset();
                }
                break;

            case 2: // Recovering - wait before resuming normal control
                if (now - stuckStateStartTime > reverseRecoveryTime) {
                    // Resume normal operation
                    stuckDetectionState = 0;
                    lastErrorChangeTime = now;
                    targetTicks += Convertor.toTicks(reverseDistance);
                    sorterPID.reset();
                }
                break;
        }
    }

    // ================= COMMANDS =================
    public void moveDegrees(double degrees) {
        targetTicks += Convertor.toTicks(degrees);
        sorterPID.reset();
        stableSince = 0;
        lastErrorChangeTime = System.currentTimeMillis();
        stuckDetectionState = 0;
    }

    public void resetPID() {
        sorterPID.reset();
        hw.sorter.setPower(0);
        targetTicks = hw.sorter.getCurrentPosition();
        stableSince = 0;
        lastTime = System.currentTimeMillis();
        lastErrorChangeTime = System.currentTimeMillis();
        stuckDetectionState = 0;
    }

    // ================= TARGET CHECK =================
    public boolean atTarget() {
        double error = targetTicks - hw.sorter.getCurrentPosition();

        if (Math.abs(error) < targetTolerance) {
            if (stableSince == 0) {
                stableSince = System.currentTimeMillis();
            }
            return (System.currentTimeMillis() - stableSince) > settleTime;
        } else {
            stableSince = 0;
            return false;
        }
    }

    // ================= TRANSFER SERVO =================
    public boolean transferArtifact(double currentTime) {
        switch (transferState) {
            case 0:
                hw.sorterTransfer.setPosition(RobotHardware.transferPush);
                transferStartTime = currentTime;
                transferState = 1;
                return false;

            case 1:
                if (currentTime - transferStartTime > 0.5) {
                    hw.sorterTransfer.setPosition(RobotHardware.transferIdle);
                    transferStartTime = currentTime;
                    transferState = 2;
                }
                return false;

            case 2:
                if (currentTime - transferStartTime > 0.5) {
                    transferState = 0;
                    return true;
                }
                return false;

            default:
                transferState = 0;
                return true;
        }
    }
}