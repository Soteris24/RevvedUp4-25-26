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
    public static double kP = 0.0004; //0.0003
    public static double kI = 0.0;
    public static double kD = 0.000008; //0.000004

    // ===== CONTROL PARAMS =====
    public static double minPower = 0.1;   // overcome static friction
    public static int slowZone = 50;        // ticks
    public static int targetTolerance = 200;  // ticks
    public static long settleTime = 50;      // ms

    public int targetTicks;
    private long lastTime;

    private long stableSince = 0;

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
    }

    // ================= UPDATE LOOP =================
    public void update() {
        long now = System.currentTimeMillis();
        double dt = Math.max((now - lastTime) / 1000.0, 0.001);
        lastTime = now;

        sorterPID.setCoefficients(new PIDCoefficients(kP , kI, kD));

        int position = hw.sorter.getCurrentPosition();
        double error = targetTicks - position;

        double power = sorterPID.update(error, dt);

        // Slow down near target
        double scale = Math.min(1.0, Math.abs(error) / slowZone);
        power *= scale;

        // Minimum power clamp (prevents stalling & wiggle)
        if (Math.abs(power) < minPower && Math.abs(error) > targetTolerance) {
            power = Math.signum(power) * minPower;
        }

        power = Math.max(-1.0, Math.min(1.0, power));
        hw.sorter.setPower(power);

        if (telemetryOn) {
            panelsTelemetry.getTelemetry().addData("Sorter Pos", position);
            panelsTelemetry.getTelemetry().addData("Target", targetTicks);
            panelsTelemetry.getTelemetry().addData("Error", error);
            panelsTelemetry.getTelemetry().addData("Power", power);
        }
    }

    // ================= COMMANDS =================
    public void moveDegrees(double degrees) {
        targetTicks += Convertor.toTicks(degrees);
        sorterPID.reset();
        stableSince = 0;
    }

    public void resetPID() {
        sorterPID.reset();
        hw.sorter.setPower(0);
        targetTicks = hw.sorter.getCurrentPosition();
        stableSince = 0;
        lastTime = System.currentTimeMillis();
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
