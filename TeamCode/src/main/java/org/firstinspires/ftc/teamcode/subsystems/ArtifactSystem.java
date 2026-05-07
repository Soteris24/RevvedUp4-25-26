package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.Arrays;
import java.util.Objects;

public class ArtifactSystem {

    private final RobotHardware hw;
    private final Sorter sorter;
    private final Shooter2 shooter;
    private final Intake intake;
    private final Telemetry telemetry;
    private final PanelsTelemetry panelsTel = PanelsTelemetry.INSTANCE;
    private final boolean telemetryOn;

    public enum RobotState { INTAKE, SHOOTING, MANUAL }
    public enum ShootSubState { IDLE, MOVE_TO_SLOT, ROTATE_SORTER, TRANSFER, RESET }
    public enum IntakeSubState { IDLE, INTAKE, WAIT, RETURN, FULL }

    public RobotState robotState = RobotState.INTAKE;
    public ShootSubState shootSubState = ShootSubState.IDLE;
    public IntakeSubState intakeSubState = IntakeSubState.IDLE;

    public String[] storedArtifacts = new String[3];
    public int artifactCount = 0;
    public int targetSlot = 0;
    public int currentSlot = 0;
    public int nextSlotIndex = 0;
    public boolean offSetApplied = false;
    public boolean artifactPresent = false;

    public String[] motif = {"G", "P", "P"};
    private int motifProgress = 0;

    public double rpm = SHORT_RPM;
    public static final double SHORT_RPM = 2030;
    public static final double LONG_RPM = 2450;
    public double currentDistance = 48;

    private static final double[] DISTANCE_TABLE = {24, 48, 72, 96};
    private static final double[] RPM_TABLE = {2000, 2100, 2300, 2500};

    private final int[] insertionOrder = new int[3];
    private int insertHead = 0;
    private int insertTail = 0;

    private String pendingShootColor = null;
    private boolean transferInProgress = false;
    private double transferStartTime = 0;
    private boolean sorterMoved = false;
    private boolean autoFire = false;

    private double intakeRotateStartTime = 0;
    private double rotateStartTime = 0;
    private static final double INTAKE_SETTLE_SEC = 0.1;
    private static final double ROTATE_GUARD_SEC = 0.15;

    private boolean manualTransferActive = false;
    private double manualTransferStart = 0;
    private static final double MANUAL_TRANSFER_SEC = 0.3;
    public double shootPhase1 = 0.14;
    public double shootPhase2 = 0.15;
    boolean dynamicShoot = false;

    private boolean lastDetected = false;
    private long lastDetectionTime = 0;
    private long lastSlotSwitchTime = 0;
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

    public void update(double currentTime) { //, boolean ltEdge, boolean rtEdge, boolean servoEdge
        if (robotState == RobotState.INTAKE) {
            detect();
        }

        switch (robotState) {
            case INTAKE:
                updateIntake(currentTime);
                break;
            case SHOOTING:
                updateShooting(currentTime);
                break;
            case MANUAL:
                //updateManual(ltEdge, rtEdge, servoEdge, currentTime);
                //pekse mucho
                break;
        }

        if (telemetryOn) {
            panelsTel.getTelemetry().addData("RobotState", robotState);
            panelsTel.getTelemetry().addData("ShootSubState", shootSubState);
            panelsTel.getTelemetry().addData("IntakeSubState", intakeSubState);
            panelsTel.getTelemetry().addData("Artifacts", Arrays.toString(storedArtifacts));
            panelsTel.getTelemetry().addData("ArtifactCount", artifactCount);
            panelsTel.getTelemetry().addData("CurrentSlot", currentSlot);
            panelsTel.getTelemetry().addData("TargetSlot", targetSlot);
            panelsTel.getTelemetry().addData("RPM", rpm);
            panelsTel.getTelemetry().addData("ManualTransferActive", manualTransferActive);
        }
    }

    public void switchToShooting(double targetRpm, boolean dynamic) {
        rpm = dynamic ? getRPMForDistance(currentDistance) : targetRpm;
        dynamicShoot = dynamic;
        if (robotState != RobotState.SHOOTING) {
            enterShootingState();
        } else {
            shooter.setTargetVelRPM(rpm);
        }
    }

    public void switchToIntake() {
        if (robotState != RobotState.INTAKE) {
            enterIntakeState();
        }
    }

    public void switchToManual() {
        if (robotState != RobotState.MANUAL) {
            enterManualState();
        }
    }

    public void toggleIntake() {
        if (robotState != RobotState.INTAKE) {
            return;
        }

        intake.intakeOn = !intake.intakeOn;
        intakeSubState = intake.intakeOn ? IntakeSubState.INTAKE : IntakeSubState.IDLE;
        if (intake.intakeOn) {
            artifactPresent = false;
        }
    }

    public void setIntakeReverse(boolean reverse, double currentTime) {
        if (robotState == RobotState.INTAKE) {
            intake.intake(false, reverse, currentTime);
        }
    }

    public void triggerAutoFire() {
        if (robotState != RobotState.SHOOTING || shootSubState != ShootSubState.IDLE || artifactCount <= 0) {
            return;
        }

        autoFire = true;
        pendingShootColor = null;
        startNextAutoShot();
    }

    public void triggerColorShot(String color) {
        if (robotState != RobotState.SHOOTING || shootSubState != ShootSubState.IDLE) {
            return;
        }

        pendingShootColor = color;
        startShot();
    }

    public void triggerManualShot(double currentTime) {
        if (manualTransferActive) {
            return;
        }
        if (robotState == RobotState.MANUAL && !sorter.atTarget()) {
            return;
        }

        manualTransferActive = true;
        manualTransferStart = currentTime;
        hw.sorterTransfer.setPosition(RobotHardware.transferPush);
    }

    public void manualDetect(String color) {
        if (robotState != RobotState.INTAKE || artifactCount >= storedArtifacts.length) {
            return;
        }
        if (System.currentTimeMillis() - lastSlotSwitchTime < SLOT_DEBOUNCE_MS) {
            return;
        }

        if (storeArtifact(color)) {
            artifactPresent = true;
            lastSlotSwitchTime = System.currentTimeMillis();
        }
    }

    public void inspectSlot(String color) {
        if (robotState != RobotState.INTAKE) {
            return;
        }
        if (System.currentTimeMillis() - lastSlotSwitchTime < SLOT_DEBOUNCE_MS) {
            return;
        }

        int slot = findNextSlotByColor(color, inspectSlotIndex);
        if (slot == -1) {
            return;
        }

        rotateToSlot(slot, false);
        inspectSlotIndex = (slot + 1) % storedArtifacts.length;
        lastSlotSwitchTime = System.currentTimeMillis();
    }

    public void manualRotate(int direction) {
        if (robotState != RobotState.MANUAL || manualTransferActive) {
            return;
        }
        if (System.currentTimeMillis() - lastSlotSwitchTime < SLOT_DEBOUNCE_MS) {
            return;
        }

        sorter.moveDegrees(direction * RobotHardware.DEG_PER_SLOT);
        lastSlotSwitchTime = System.currentTimeMillis();
    }

    public void seedArtifacts(String... colors) {
        resetSlot();
        int count = Math.min(colors.length, storedArtifacts.length);
        for (int i = 0; i < count; i++) {
            if (colors[i] == null) {
                continue;
            }
            storeArtifact(colors[i]);
        }
        artifactPresent = false;
    }

    private void updateIntake(double currentTime) {
        switch (intakeSubState) {
            case INTAKE:
                if (artifactPresent) {
                    intakeRotateStartTime = currentTime;
                    intake.slow();
                    intakeSubState = IntakeSubState.WAIT;
                }
                break;

            case WAIT:
                if (currentTime - intakeRotateStartTime >= INTAKE_SETTLE_SEC) {
                    if (artifactCount < storedArtifacts.length) {
                        sorter.moveDegrees(RobotHardware.DEG_PER_SLOT);
                        intakeSubState = IntakeSubState.RETURN;
                    } else {
                        intakeSubState = IntakeSubState.FULL;
                    }
                }
                break;

            case RETURN:
                if (sorter.atTarget()) {
                    intake.intakeOn = true;
                    artifactPresent = false;
                    intakeSubState = artifactCount >= storedArtifacts.length ? IntakeSubState.FULL : IntakeSubState.INTAKE;
                }
                break;

            case FULL:
                intake.intakeOn = false;
                if (sorter.atTarget()) {
                    intakeSubState = IntakeSubState.IDLE;
                }
                break;

            case IDLE:
            default:
                break;
        }

        shooter.setTargetVelRPM(intake.intakeOn ? 1000 : 0);
    }

    private void updateShooting(double currentTime) {
        if (dynamicShoot) {
            rpm = getRPMForDistance(currentDistance);
        }
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
                if (currentTime - rotateStartTime < ROTATE_GUARD_SEC) {
                    break;
                }
                if (!sorterMoved && sorter.atTarget()) {
                    rotateToSlot(targetSlot, true);
                    sorterMoved = true;
                } else if (sorterMoved && sorter.atTarget()) {
                    transferInProgress = true;
                    shootSubState = ShootSubState.TRANSFER;
                }
                break;

            case TRANSFER:
                if (transferInProgress && sorter.atTarget() && (shooter.atTargetVel() || currentTime - rotateStartTime > 3.0)) {
                    transferStartTime = currentTime;
                    hw.sorterTransfer.setPosition(RobotHardware.transferPush);
                    shootSubState = ShootSubState.RESET;
                }
                break;

            case RESET:
                if (transferInProgress && currentTime - transferStartTime > shootPhase1) {
                    hw.sorterTransfer.setPosition(RobotHardware.transferIdle);
                    removeArtifact(currentSlot);
                    transferInProgress = false;
                    sorterMoved = false;
                    pendingShootColor = null;
                    transferStartTime = currentTime;
                }
                if (!transferInProgress && currentTime - transferStartTime > shootPhase2) {
                    if (artifactCount <= 0) {
                        autoFire = false;
                        enterIntakeState();
                    } else if (autoFire) {
                        startNextAutoShot();
                    } else {
                        shootSubState = ShootSubState.IDLE;
                    }
                }
                break;

            case IDLE:
            default:
                break;
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
                manualTransferStart = currentTime;
                hw.sorterTransfer.setPosition(RobotHardware.transferPush);
            }
        } else if (currentTime - manualTransferStart > MANUAL_TRANSFER_SEC) {
            hw.sorterTransfer.setPosition(RobotHardware.transferIdle);
            manualTransferActive = false;
        }
    }

    private void enterIntakeState() {
        shooter.setTargetVelRPM(0);
        intake.intakeOn = false;
        manualTransferActive = false;
        hw.sorterTransfer.setPosition(RobotHardware.transferIdle);

        if (offSetApplied) {
            sorter.moveDegrees(55);
        }

        transferInProgress = false;
        sorterMoved = false;
        pendingShootColor = null;
        autoFire = false;
        inspectSlotIndex = 0;
        intakeSubState = IntakeSubState.IDLE;
        shootSubState = ShootSubState.IDLE;
        robotState = RobotState.INTAKE;
        dynamicShoot = false;

        resetSlot();
    }

    private void enterShootingState() {
        robotState = RobotState.SHOOTING;
        shootSubState = ShootSubState.IDLE;
        intake.stop();
        sorterMoved = false;
        transferInProgress = false;
        pendingShootColor = null;
        manualTransferActive = false;
        autoFire = false;
        shooter.setTargetVelRPM(rpm);
    }

    private void enterManualState() {
        robotState = RobotState.MANUAL;
        shootSubState = ShootSubState.IDLE;
        manualTransferActive = false;
    }

    private void rotateToSlot(int slot, boolean requireArtifact) {
        if (slot < 0 || slot >= storedArtifacts.length) {
            return;
        }
        if (requireArtifact && storedArtifacts[slot] == null) {
            return;
        }

        if (!offSetApplied) {
            double[] initialOffsets = {-55, 65, 185};
            sorter.moveDegrees(initialOffsets[slot]);
            offSetApplied = true;
            currentSlot = slot;
            return;
        }

        int delta = (slot - currentSlot + storedArtifacts.length) % storedArtifacts.length;
        int step = delta == 1 ? 1 : delta == 2 ? -1 : 0;
        if (step != 0) {
            sorter.moveDegrees(step * RobotHardware.DEG_PER_SLOT);
        }
        currentSlot = slot;
    }

    private void startNextAutoShot() {
        while (insertHead < insertTail) {
            int slot = insertionOrder[insertHead % insertionOrder.length];
            insertHead++;
            if (slot >= 0 && slot < storedArtifacts.length && storedArtifacts[slot] != null) {
                targetSlot = slot;
                pendingShootColor = storedArtifacts[slot];
                shootSubState = ShootSubState.MOVE_TO_SLOT;
                return;
            }
        }

        int fallbackSlot = findAnySlot();
        if (fallbackSlot != -1) {
            targetSlot = fallbackSlot;
            pendingShootColor = storedArtifacts[fallbackSlot];
            shootSubState = ShootSubState.MOVE_TO_SLOT;
            return;
        }

        autoFire = false;
        enterIntakeState();
    }

    private void startShot() {
        int slot = findSlotByColor(pendingShootColor);
        if (slot == -1) {
            slot = findAnySlot();
        }
        if (slot == -1) {
            pendingShootColor = null;
            return;
        }

        targetSlot = slot;
        shootSubState = ShootSubState.MOVE_TO_SLOT;
    }

    private void detect() {
        double distance = hw.colorSensor.getDistance(DistanceUnit.MM);
        boolean detectedNow = distance < 85 && distance > 30;
        long now = System.currentTimeMillis();

        if (!artifactPresent && detectedNow && !lastDetected && now - lastDetectionTime > 300) {
            if (artifactCount < storedArtifacts.length) {
                String color = hw.colorSensor.green() > hw.colorSensor.blue() ? "G" : "P";
                if (storeArtifact(color)) {
                    artifactPresent = true;
                    lastDetectionTime = now;
                }
            }
        }

        lastDetected = detectedNow;
    }

    private boolean storeArtifact(String color) {
        if (artifactCount >= storedArtifacts.length) {
            return false;
        }

        int slot = findNextEmptySlot(nextSlotIndex);
        if (slot == -1) {
            syncArtifactCount();
            return false;
        }

        storedArtifacts[slot] = color;
        insertionOrder[insertTail % insertionOrder.length] = slot;
        insertTail++;
        nextSlotIndex = (slot + 1) % storedArtifacts.length;
        syncArtifactCount();
        return true;
    }

    private int findSlotByColor(String color) {
        for (int i = 0; i < storedArtifacts.length; i++) {
            if (Objects.equals(storedArtifacts[i], color)) {
                return i;
            }
        }
        return -1;
    }

    private int findNextSlotByColor(String color, int start) {
        for (int i = 0; i < storedArtifacts.length; i++) {
            int idx = (start + i) % storedArtifacts.length;
            if (Objects.equals(storedArtifacts[idx], color)) {
                return idx;
            }
        }
        return -1;
    }

    private int findAnySlot() {
        for (int i = 0; i < storedArtifacts.length; i++) {
            if (storedArtifacts[i] != null) {
                return i;
            }
        }
        return -1;
    }

    private void removeArtifact(int slot) {
        if (slot < 0 || slot >= storedArtifacts.length) {
            return;
        }
        if (storedArtifacts[slot] != null) {
            storedArtifacts[slot] = null;
            motifProgress = (motifProgress + 1) % motif.length;
            nextSlotIndex = slot;
            syncArtifactCount();
        }
    }

    private int findNextEmptySlot(int start) {
        for (int i = 0; i < storedArtifacts.length; i++) {
            int idx = (start + i) % storedArtifacts.length;
            if (storedArtifacts[idx] == null) {
                return idx;
            }
        }
        return -1;
    }

    private void syncArtifactCount() {
        int count = 0;
        for (String artifact : storedArtifacts) {
            if (artifact != null) {
                count++;
            }
        }
        artifactCount = count;
    }

    public void resetSlot() {
        currentSlot = 0;
        artifactCount = 0;
        nextSlotIndex = 0;
        motifProgress = 0;
        offSetApplied = false;
        artifactPresent = false;
        insertHead = 0;
        insertTail = 0;
        Arrays.fill(storedArtifacts, null);
        Arrays.fill(insertionOrder, 0);
    }

    public boolean isActivelyShooting() {
        return robotState == RobotState.SHOOTING && shootSubState != ShootSubState.IDLE;
    }

    private double getRPMForDistance(double distance) {
        if (distance <= DISTANCE_TABLE[0]) {
            return RPM_TABLE[0];
        }
        if (distance >= DISTANCE_TABLE[DISTANCE_TABLE.length - 1]) {
            return RPM_TABLE[RPM_TABLE.length - 1];
        }

        for (int i = 0; i < DISTANCE_TABLE.length - 1; i++) {
            if (distance >= DISTANCE_TABLE[i] && distance <= DISTANCE_TABLE[i + 1]) {
                double t = (distance - DISTANCE_TABLE[i]) / (DISTANCE_TABLE[i + 1] - DISTANCE_TABLE[i]);
                return RPM_TABLE[i] + t * (RPM_TABLE[i + 1] - RPM_TABLE[i]);
            }
        }

        return RPM_TABLE[RPM_TABLE.length - 1];
    }
}
