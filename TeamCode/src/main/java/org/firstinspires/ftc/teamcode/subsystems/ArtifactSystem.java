package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import java.util.Arrays;
import java.util.Objects;

public class ArtifactSystem {

    private final RobotHardware   hw;
    private final Sorter          sorter;
    private final Shooter2        shooter;
    private final Intake          intake;
    private final Telemetry       telemetry;
    private final PanelsTelemetry panelsTel = PanelsTelemetry.INSTANCE;
    private final boolean         telemetryOn;

    public enum RobotState     { INTAKE, SHOOTING, MANUAL }
    public enum ShootSubState  { IDLE, MOVE_TO_SLOT, ROTATE_SORTER, TRANSFER, RESET }
    public enum IntakeSubState { IDLE, INTAKE, WAIT, RETURN, FULL }

    public RobotState     robotState     = RobotState.INTAKE;
    public ShootSubState  shootSubState  = ShootSubState.IDLE;
    public IntakeSubState intakeSubState = IntakeSubState.IDLE;

    public String[]  storedArtifacts = new String[3];
    public int       artifactCount   = 0;
    public int       targetSlot      = 0;
    public int       currentSlot     = 0;
    public int      nextSlotIndex   = 0;
    public boolean   offSetApplied   = false;
    public boolean   artifactPresent = false;

    public String[] motif         = {"G", "P", "P"};
    private int     motifProgress = 0;

    public double rpm                    = SHORT_RPM;
    public static final double SHORT_RPM = 2030;
    public static final double LONG_RPM  = 2450;
    public double currentDistance        = 48;

    private static final double[] DISTANCE_TABLE = { 24,   48,   72,   96   };
    private static final double[] RPM_TABLE      = { 2000, 2100, 2300, 2500 };

    private String  pendingShootColor  = null;
    private boolean transferInProgress = false;
    private double  transferStartTime  = 0;
    private boolean sorterMoved        = false;
    private boolean autoFire           = false;

    private double intakeRotateStartTime = 0;
    private double rotateStartTime       = 0;
    private static final double ROTATE_GUARD_SEC = 0.15;

    private boolean manualTransferActive  = false;
    private double  manualTransferStart   = 0;
    private static final double MANUAL_TRANSFER_SEC = 0.4;
    public double shootPhase1 = 0.18;
    public double shootPhase2 = 0.18;
    boolean dynamicShoot = false;

    private boolean lastDetected      = false;
    private long    lastDetectionTime = 0;
    private long    lastSlotSwitchTime = 0;
    private static final long SLOT_DEBOUNCE_MS = 300;
    private int inspectSlotIndex = 0;

    public ArtifactSystem(RobotHardware hw, Telemetry telemetry, Sorter sorter, Shooter2 shooter, Intake intake, boolean telemetryOn) {
        this.hw = hw;
        this.telemetry = telemetry;
        this.sorter = sorter;
        this.shooter = shooter;
        this.intake = intake;
        this.telemetryOn = telemetryOn;
    }


    public void update(double currentTime, boolean ltEdge, boolean rtEdge, boolean servoEdge) {
        detect();

        switch (robotState) {
            case INTAKE:   updateIntake(currentTime);   break;
            case SHOOTING: updateShooting(currentTime); break;
            case MANUAL:   updateManual(ltEdge, rtEdge, servoEdge, currentTime);   break;
        }

        if (telemetryOn) {
            panelsTel.getTelemetry().addData("State", robotState + " | " + shootSubState);
            panelsTel.getTelemetry().addData("Artifacts", Arrays.toString(storedArtifacts));
            panelsTel.getTelemetry().addData("Count", artifactCount);
            panelsTel.getTelemetry().addData("RPM", rpm);
        }
    }

    public void switchToShooting(double targetRpm, boolean dynamic) {
        this.rpm = targetRpm;
        this.dynamicShoot = dynamic;
        if (robotState != RobotState.SHOOTING) enterShootingState();
        shooter.setTargetVelRPM(rpm);
    }

    public void switchToIntake() {
        if (robotState != RobotState.INTAKE) enterIntakeState();
    }

    public void switchToManual() {
        if (robotState != RobotState.MANUAL) enterManualState();
    }

    public void toggleIntake() {
        if (robotState != RobotState.INTAKE) return;
        intake.intakeOn = !intake.intakeOn;
        intakeSubState = intake.intakeOn ? IntakeSubState.INTAKE : IntakeSubState.IDLE;
        if (intake.intakeOn) artifactPresent = false;
    }

    public void setIntakeReverse(boolean reverse, double currentTime) {
        intake.intake(false, reverse, currentTime);
    }

    public void triggerAutoFire() {
        if (robotState == RobotState.SHOOTING && shootSubState == ShootSubState.IDLE && artifactCount > 0) {
            autoFire = true;
            startNextAutoShot();
        }
    }

    public void triggerColorShot(String color) {
        if (robotState != RobotState.SHOOTING || shootSubState != ShootSubState.IDLE) return;
        pendingShootColor = color;
        startShot();
    }

    public void triggerManualShot(double currentTime) {
        if (manualTransferActive) return;
        if (robotState == RobotState.MANUAL && !sorter.atTarget()) return;

        manualTransferActive = true;
        manualTransferStart  = currentTime;
        hw.sorterTransfer.setPosition(RobotHardware.transferPush);
    }

    public void manualDetect(String color) {
        if (artifactCount >= storedArtifacts.length) return;
        storedArtifacts[nextSlotIndex] = color;
        nextSlotIndex = (nextSlotIndex + 1) % 3;
        artifactPresent = true;
        lastSlotSwitchTime = System.currentTimeMillis();
    }

    public void inspectSlot(String color) {
        if (System.currentTimeMillis() - lastSlotSwitchTime < SLOT_DEBOUNCE_MS) return;
        int slot = findNextSlotByColor(color, inspectSlotIndex);
        if (slot != -1) {
            rotateToSlot(slot, false);
            inspectSlotIndex = (slot + 1) % 3;
            lastSlotSwitchTime = System.currentTimeMillis();
        }
    }

    public void manualRotate(int direction) {
        if (manualTransferActive || System.currentTimeMillis() - lastSlotSwitchTime < SLOT_DEBOUNCE_MS) return;
        sorter.moveDegrees(direction * RobotHardware.DEG_PER_SLOT);
        lastSlotSwitchTime = System.currentTimeMillis();
    }

    // INTERNAL STATE MACHINES

    private void updateIntake(double currentTime) {
        intake.intake(false, false, currentTime);

        switch (intakeSubState) {
            case INTAKE:
                if (artifactPresent) {
                    intakeRotateStartTime = currentTime;
                    intakeSubState = IntakeSubState.WAIT;
                }
                break;
            case WAIT:
                if (currentTime - intakeRotateStartTime >= 0.1) {
                    if (artifactCount < 2) {
                        sorter.moveDegrees(120);
                        intakeSubState = IntakeSubState.RETURN;
                    } else {
                        intakeSubState = IntakeSubState.FULL;
                    }
                }
                break;
            case RETURN:
                intake.slow();
                if (sorter.atTarget()) {
                    artifactCount++;
                    artifactPresent = false;
                    intakeSubState = (artifactCount >= 3) ? IntakeSubState.FULL : IntakeSubState.INTAKE;
                }
                break;
            case FULL:
                intake.intakeOn = false;
                break;
            default: break;
        }
        shooter.setTargetVelRPM(intake.intakeOn ? 1000 : 0);
    }

    private void updateShooting(double currentTime) {
        if (dynamicShoot) rpm = getRPMForDistance(currentDistance);
        shooter.setTargetVelRPM(rpm);

        if (manualTransferActive) {
            if (currentTime - manualTransferStart > MANUAL_TRANSFER_SEC) {
                hw.sorterTransfer.setPosition(RobotHardware.transferIdle);
                manualTransferActive = false;
            }
            return;
        }

        switch (shootSubState) {
            case MOVE_TO_SLOT:
                rotateToSlot(targetSlot, true);
                sorterMoved = false;
                rotateStartTime = currentTime;
                shootSubState = ShootSubState.ROTATE_SORTER;
                break;
            case ROTATE_SORTER:
                if (currentTime - rotateStartTime < ROTATE_GUARD_SEC) break;
                if (!sorterMoved && sorter.atTarget()) {
                    rotateToSlot(targetSlot, true);
                    sorterMoved = true;
                } else if (sorterMoved && sorter.atTarget()) {
                    transferInProgress = true;
                    shootSubState = ShootSubState.TRANSFER;
                }
                break;
            case TRANSFER:
                if (transferInProgress && sorter.atTarget() && (shooter.atTargetVel() || (currentTime - rotateStartTime > 3.0))) {
                    transferStartTime = currentTime;
                    hw.sorterTransfer.setPosition(RobotHardware.transferPush);
                    shootSubState = ShootSubState.RESET;
                }
                break;
            case RESET:
                if (transferInProgress && currentTime - transferStartTime > shootPhase1) {
                    hw.sorterTransfer.setPosition(RobotHardware.transferIdle);
                    removeArtifact(currentSlot);
                    artifactCount--;
                    transferInProgress = false;
                    sorterMoved = false;
                    transferStartTime = currentTime;
                }
                if (!transferInProgress && currentTime - transferStartTime > shootPhase2) {
                    if (artifactCount <= 0) {
                        enterIntakeState();
                    } else if (autoFire) {
                        startNextAutoShot();
                    } else {
                        shootSubState = ShootSubState.IDLE;
                    }
                }
                break;
            default: break;
        }
    }

    private void updateManual(boolean ltEdge, boolean rtEdge, boolean servoEdge, double currentTime) {
        long now = System.currentTimeMillis();

        if (!manualTransferActive) {
            if (now - lastSlotSwitchTime > SLOT_DEBOUNCE_MS) {
                if (ltEdge) {
                    sorter.moveDegrees(-RobotHardware.DEG_PER_SLOT);
                    lastSlotSwitchTime = now;
                } else if (rtEdge) {
                    sorter.moveDegrees(RobotHardware.DEG_PER_SLOT);
                    lastSlotSwitchTime = now;
                }
            }

            if (servoEdge && sorter.atTarget()) {
                manualTransferActive = true;
                manualTransferStart  = currentTime;
                hw.sorterTransfer.setPosition(RobotHardware.transferPush);
            }
        } else {
            if (currentTime - manualTransferStart > MANUAL_TRANSFER_SEC) {
                hw.sorterTransfer.setPosition(RobotHardware.transferIdle);
                manualTransferActive = false;
            }
        }
    }

    // =========================================================================
    // HELPERS
    // =========================================================================

    private void enterIntakeState() {
        shooter.setTargetVelRPM(0);
        intake.intakeOn = false;
        hw.sorterTransfer.setPosition(RobotHardware.transferIdle);

        if (offSetApplied) {
            sorter.moveDegrees(55); 
        }

        robotState = RobotState.INTAKE;
        intakeSubState = IntakeSubState.IDLE;
        shootSubState = ShootSubState.IDLE;
        offSetApplied = false;
        resetSlot();
    }

    private void enterShootingState() {
        robotState = RobotState.SHOOTING;
        shootSubState = ShootSubState.IDLE;
        intake.stop();
        autoFire = false;
    }

    private void enterManualState() {
        robotState = RobotState.MANUAL;
    }

    private void rotateToSlot(int slot, boolean checkArtifact) {
        if (checkArtifact && storedArtifacts[slot] == null) return;
        if (!offSetApplied) {
            double[] initialOffsets = {-55, 65, 185};
            sorter.moveDegrees(initialOffsets[slot]);
            offSetApplied = true;
            currentSlot = slot;
            return;
        }
        int delta = (slot - currentSlot + 3) % 3;
        int step = (delta == 1) ? 1 : (delta == 2) ? -1 : 0;
        if (step != 0) sorter.moveDegrees(step * RobotHardware.DEG_PER_SLOT);
        currentSlot = slot;
    }

    private void startNextAutoShot() {
        int slot = findSlotByColor(motif[motifProgress]);
        if (slot == -1) slot = findAnySlot();
        if (slot == -1) { autoFire = false; enterIntakeState(); return; }
        targetSlot = slot;
        shootSubState = ShootSubState.MOVE_TO_SLOT;
    }

    private void startShot() {
        int slot = findSlotByColor(pendingShootColor);
        if (slot == -1) slot = findAnySlot();
        if (slot == -1) return;
        targetSlot = slot;
        shootSubState = ShootSubState.MOVE_TO_SLOT;
    }

    private void detect() {
        double d2 = hw.colorSensor.getDistance(DistanceUnit.MM);
        boolean detectedNow = (d2 < 85 && d2 > 30);
        long now = System.currentTimeMillis();
        if (!artifactPresent && detectedNow && !lastDetected && now - lastDetectionTime > 300) {
            artifactPresent = true;
            String color = (hw.colorSensor.green() > hw.colorSensor.blue()) ? "G" : "P";
            if (artifactCount < 3) {
                storedArtifacts[nextSlotIndex] = color;
                nextSlotIndex = (nextSlotIndex + 1) % 3;
            }
            lastDetectionTime = now;
        }
        lastDetected = detectedNow;
    }

    private int findSlotByColor(String color) {
        for (int i = 0; i < 3; i++) if (Objects.equals(storedArtifacts[i], color)) return i;
        return -1;
    }

    private int findNextSlotByColor(String color, int start) {
        for (int i = 0; i < 3; i++) {
            int idx = (start + i) % 3;
            if (Objects.equals(storedArtifacts[idx], color)) return idx;
        }
        return -1;
    }

    private int findAnySlot() {
        for (int i = 0; i < 3; i++) if (storedArtifacts[i] != null) return i;
        return -1;
    }

    private void removeArtifact(int slot) {
        if (storedArtifacts[slot] != null) {
            storedArtifacts[slot] = null;
            motifProgress = (motifProgress + 1) % motif.length;
        }
    }

    public void resetSlot() {
        currentSlot = 0; artifactCount = 0; nextSlotIndex = 0; motifProgress = 0;
        Arrays.fill(storedArtifacts, null);
    }

    public boolean isActivelyShooting() {
        return robotState == RobotState.SHOOTING && shootSubState != ShootSubState.IDLE;
    }

    private double getRPMForDistance(double distance) {
        if (distance <= DISTANCE_TABLE[0]) return RPM_TABLE[0];
        if (distance >= DISTANCE_TABLE[3]) return RPM_TABLE[3];
        for (int i = 0; i < 3; i++) {
            if (distance >= DISTANCE_TABLE[i] && distance <= DISTANCE_TABLE[i+1]) {
                double t = (distance - DISTANCE_TABLE[i]) / (DISTANCE_TABLE[i+1] - DISTANCE_TABLE[i]);
                return RPM_TABLE[i] + t * (RPM_TABLE[i+1] - RPM_TABLE[i]);
            }
        }
        return RPM_TABLE[3];
    }
}
