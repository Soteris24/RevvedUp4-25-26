package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.Arrays;
import java.util.Objects;

/**
 * ======================== BUTTON MAP (gamepad2) ========================
 *
 * INTAKE STATE:
 *   Y                  — toggle intake on/off
 *   A                  — reverse intake (hold)
 *   left_bumper        — manual GREEN  detect (mark current ball as G)
 *   right_bumper       — manual PURPLE detect (mark current ball as P)
 *   left_trigger       — cycle sorter to next GREEN  slot (fix/inspect)
 *   right_trigger      — cycle sorter to next PURPLE slot (fix/inspect)
 *   dpad_left          — enter SHOOT state (short RPM)
 *   dpad_up            — enter SHOOT state (long RPM)
 *
 * SHOOT STATE:
 *   left_bumper        — find GREEN  artifact, rotate sorter, fire, retract
 *   right_bumper       — find PURPLE artifact, rotate sorter, fire, retract
 *   X (IDLE only)      — shoot current slot (no rotation, fires what is there now)
 *   X (mid-shot)       — rescue: abort shot and force servo back to idle
 *   dpad_up            — SET long RPM  (2700)
 *   dpad_down          — SET short RPM (2100)
 *   dpad_right         — exit to INTAKE
 *
 * MANUAL STATE:
 *   left_trigger       — rotate sorter one step left  (blocked while servo active)
 *   right_trigger      — rotate sorter one step right (blocked while servo active)
 *   Y                  — fire current slot             (blocked while sorter moving)
 *   dpad_right         — exit to INTAKE
 *
 * ======================================================================
 */
public class ArtifactSystem {

    // =========================================================================
    // DEPENDENCIES
    // =========================================================================
    private final RobotHardware   hw;
    private final Sorter          sorter;
    private final Shooter2        shooter;
    private final Intake          intake;
    private final Telemetry       telemetry;
    private final PanelsTelemetry panelsTel = PanelsTelemetry.INSTANCE;
    private final boolean         telemetryOn;

    // =========================================================================
    // PUBLIC STATE ENUMS
    // =========================================================================
    public enum RobotState     { INTAKE, SHOOTING, MANUAL }
    public enum ShootSubState  { IDLE, MOVE_TO_SLOT, ROTATE_SORTER, TRANSFER, RESET }
    public enum IntakeSubState { IDLE, INTAKE, WAIT, RETURN, FULL }

    public RobotState     robotState     = RobotState.INTAKE;
    public ShootSubState  shootSubState  = ShootSubState.IDLE;
    public IntakeSubState intakeSubState = IntakeSubState.IDLE;

    // =========================================================================
    // ARTIFACT STORAGE
    // =========================================================================
    public String[]  storedArtifacts = new String[3];
    public int       artifactCount   = 0;
    public int       targetSlot      = 0;
    public int       currentSlot     = 0;
    public int       nextSlotIndex   = 0;
    public boolean   offSetApplied   = false;
    public boolean   artifactPresent = false;

    // Motif sorting (optional)
    public boolean  motifSortingEnabled = false;
    public String[] motif               = {"G", "P", "P"};
    private int     motifProgress       = 0;

    // =========================================================================
    // SHOOTING INTERNALS
    // =========================================================================
    public double rpm                    = SHORT_RPM;
    public static final double SHORT_RPM = 2100;
    public static final double LONG_RPM  = 2700;

    private String  pendingShootColor  = null;
    private boolean transferInProgress = false;
    private double  transferStartTime  = 0;
    private boolean sorterMoved        = false;
    private boolean autoFire           = false; // true = fire all slots automatically

    // Guard timer — prevents ROTATE_SORTER from seeing atTarget() before
    // the motor has physically started moving.
    private double rotateStartTime               = 0;
    private static final double ROTATE_GUARD_SEC = 0.15;

    // =========================================================================
    // MANUAL TRANSFER STATE
    // Used in both MANUAL and SHOOT (IDLE) states.
    // While true:  sorter rotation is blocked.
    // While false: servo push blocked until sorter is at target.
    // =========================================================================
    private boolean manualTransferActive  = false;
    private double  manualTransferStart   = 0;
    private static final double MANUAL_TRANSFER_SEC = 0.4;

    // =========================================================================
    // COLOUR DETECTION INTERNALS
    // =========================================================================
    private boolean lastDetected      = false;
    private long    lastDetectionTime = 0;

    // =========================================================================
    // BUTTON EDGE-DETECTION HISTORY
    // =========================================================================
    private boolean lastDpadUp, lastDpadDown, lastDpadLeft, lastDpadRight;
    private boolean lastLeftTrigger, lastRightTrigger;
    private boolean lastLeftBumper,  lastRightBumper;
    private boolean lastIntakeToggle;
    private boolean lastServoTransfer;
    private boolean lastIntakeReverse;
    private boolean lastButtonB;

    // =========================================================================
    // DEBOUNCE
    // =========================================================================
    private long lastSlotSwitchTime          = 0;
    private static final long SLOT_DEBOUNCE_MS = 300;

    private int inspectSlotIndex = 0;

    // =========================================================================
    // CONSTRUCTOR
    // =========================================================================
    public ArtifactSystem(RobotHardware hw, Telemetry telemetry,
                          Sorter sorter, Shooter2 shooter, Intake intake,
                          boolean telemetryOn) {
        this.hw          = hw;
        this.telemetry   = telemetry;
        this.sorter      = sorter;
        this.shooter     = shooter;
        this.intake      = intake;
        this.telemetryOn = telemetryOn;
    }

    // =========================================================================
    // MAIN UPDATE — call every loop iteration
    // =========================================================================
    public void update(boolean dpadUp,        boolean dpadDown,
                       boolean dpadLeft,      boolean dpadRight,
                       boolean leftTrigger,   boolean rightTrigger,
                       boolean leftBumper,    boolean rightBumper,
                       boolean intakeToggle,  boolean servoTransfer,
                       boolean intakeReverse, boolean buttonB,
                       double  currentTime) {

        // --- Rising edges ---
        boolean upEdge        = dpadUp        && !lastDpadUp;
        boolean downEdge      = dpadDown      && !lastDpadDown;
        boolean leftEdge      = dpadLeft      && !lastDpadLeft;
        boolean rightEdge     = dpadRight     && !lastDpadRight;
        boolean ltEdge        = leftTrigger   && !lastLeftTrigger;
        boolean rtEdge        = rightTrigger  && !lastRightTrigger;
        boolean lbEdge        = leftBumper    && !lastLeftBumper;
        boolean rbEdge        = rightBumper   && !lastRightBumper;
        boolean intakeEdge    = intakeToggle  && !lastIntakeToggle;
        boolean servoEdge     = servoTransfer && !lastServoTransfer;
        boolean intakeRevEdge = intakeReverse && !lastIntakeReverse;
        boolean bEdge         = buttonB && !lastButtonB;

        // --- Global state transitions ---
        if (rightEdge && (robotState == RobotState.SHOOTING || robotState == RobotState.MANUAL)) {
            enterIntakeState();
        } else if (leftEdge && robotState == RobotState.INTAKE) {
            rpm = SHORT_RPM;
            enterShootingState();
        } else if (upEdge) {
            if (robotState == RobotState.INTAKE) {
                rpm = LONG_RPM;
                enterShootingState();
            } else if (robotState == RobotState.SHOOTING) {
                rpm = LONG_RPM;
                shooter.setTargetVelRPM(rpm);
            }
        } else if (bEdge && robotState == RobotState.SHOOTING && shootSubState == ShootSubState.IDLE) {
            // B = auto fire all — shoots every remaining artifact in sequence
            autoFire = true;
            pendingShootColor = null;
            startNextAutoShot();
        } else if (downEdge && robotState == RobotState.SHOOTING) {
            rpm = SHORT_RPM;
            shooter.setTargetVelRPM(rpm);
        }

        // --- Per-state update ---
        switch (robotState) {
            case INTAKE:
                updateIntake(lbEdge, rbEdge, ltEdge, rtEdge,
                        intakeEdge, intakeRevEdge, currentTime);
                break;
            case SHOOTING:
                updateShooting(lbEdge, rbEdge, servoEdge, currentTime);
                break;
            case MANUAL:
                updateManual(ltEdge, rtEdge, servoEdge, currentTime);
                break;
        }

        // --- Save button history ---
        lastDpadUp        = dpadUp;
        lastDpadDown      = dpadDown;
        lastDpadLeft      = dpadLeft;
        lastDpadRight     = dpadRight;
        lastLeftTrigger   = leftTrigger;
        lastRightTrigger  = rightTrigger;
        lastLeftBumper    = leftBumper;
        lastRightBumper   = rightBumper;
        lastIntakeToggle  = intakeToggle;
        lastServoTransfer = servoTransfer;
        lastIntakeReverse = intakeReverse;
        lastButtonB       = buttonB;

        // --- Telemetry ---
        if (telemetryOn) {
            panelsTel.getTelemetry().addData("RobotState",           robotState);
            panelsTel.getTelemetry().addData("ShootSubState",         shootSubState);
            panelsTel.getTelemetry().addData("IntakeSubState",        intakeSubState);
            panelsTel.getTelemetry().addData("ArtifactCount",         artifactCount);
            panelsTel.getTelemetry().addData("Artifacts",             Arrays.toString(storedArtifacts));
            panelsTel.getTelemetry().addData("CurrentSlot",           currentSlot);
            panelsTel.getTelemetry().addData("TargetSlot",            targetSlot);
            panelsTel.getTelemetry().addData("RPM Target",            rpm);
            panelsTel.getTelemetry().addData("IntakeOn",              intake.intakeOn);
            panelsTel.getTelemetry().addData("ManualTransferActive",  manualTransferActive);
            panelsTel.getTelemetry().addData("SorterAtTarget",        sorter.atTarget());
        }
    }

    // =========================================================================
    // INTAKE STATE
    // =========================================================================
    private void updateIntake(boolean lbEdge,     boolean rbEdge,
                              boolean ltEdge,     boolean rtEdge,
                              boolean intakeEdge, boolean intakeRevEdge,
                              double  currentTime) {

        if (intakeEdge) {
            intake.intakeOn = !intake.intakeOn;
            if (intake.intakeOn) {
                artifactPresent = false;
                intakeSubState  = IntakeSubState.INTAKE;
            } else {
                intakeSubState = IntakeSubState.IDLE;
            }
        }

        intake.intake(false, intakeRevEdge, currentTime);

        if (intakeSubState == IntakeSubState.INTAKE) {
            detect();
        }

        boolean manualOverride = false;
        long now = System.currentTimeMillis();

        if (now - lastSlotSwitchTime > SLOT_DEBOUNCE_MS) {
            if (lbEdge) {
                manualDetect("G");
                lastSlotSwitchTime = now;
                manualOverride     = true;
            } else if (rbEdge) {
                manualDetect("P");
                lastSlotSwitchTime = now;
                manualOverride     = true;
            }
        }

        if (now - lastSlotSwitchTime > SLOT_DEBOUNCE_MS) {
            if (ltEdge) {
                int slot = findNextSlotByColor("G", inspectSlotIndex);
                if (slot != -1) {
                    rotateToPhysicalSlot(slot);
                    inspectSlotIndex   = (slot + 1) % 3;
                    lastSlotSwitchTime = now;
                    manualOverride     = true;
                }
            } else if (rtEdge) {
                int slot = findNextSlotByColor("P", inspectSlotIndex);
                if (slot != -1) {
                    rotateToPhysicalSlot(slot);
                    inspectSlotIndex   = (slot + 1) % 3;
                    lastSlotSwitchTime = now;
                    manualOverride     = true;
                }
            }
        }

        if (manualOverride) return;

        switch (intakeSubState) {
            case IDLE:
                break;

            case INTAKE:
                if (!intake.intakeOn) {
                    intakeSubState = IntakeSubState.IDLE;
                    break;
                }
                if (artifactPresent) {
                    intakeSubState = IntakeSubState.WAIT;
                }
                break;

            case WAIT:
                if (artifactPresent) {
                    if (artifactCount < 2) {
                        sorter.moveDegrees(120);
                        intakeSubState = IntakeSubState.RETURN;
                    } else {
                        intakeSubState = IntakeSubState.FULL;
                    }
                }
                break;

            case RETURN:
                if (sorter.atTarget()) {
                    if (artifactCount >= 3) {
                        intakeSubState = IntakeSubState.FULL;
                    } else {
                        artifactCount++;
                        artifactPresent = false;
                        intakeSubState  = IntakeSubState.INTAKE;
                    }
                }
                break;

            case FULL:
                artifactCount   = 3;
                intake.intakeOn = false;
                if (sorter.atTarget()) {
                    intakeSubState = IntakeSubState.IDLE;
                }
                break;
        }

        shooter.setTargetVelRPM(intake.intakeOn ? 1000 : 0);
    }

    // =========================================================================
    // SHOOT STATE
    // =========================================================================
    private void updateShooting(boolean lbEdge, boolean rbEdge,
                                boolean servoEdge, double currentTime) {

        shooter.setTargetVelRPM(rpm);

        // X = full servo cycle (push down, wait, come back up) same as MANUAL state.
        // Works whether idle or mid-shot — just kicks the servo regardless.
        if (servoEdge && !manualTransferActive) {
            manualTransferActive = true;
            manualTransferStart  = currentTime;
            hw.sorterTransfer.setPosition(RobotHardware.transferPush);
        }
        if (manualTransferActive) {
            if (currentTime - manualTransferStart > MANUAL_TRANSFER_SEC) {
                hw.sorterTransfer.setPosition(RobotHardware.transferIdle);
                manualTransferActive = false;
            }
            return; // let the servo cycle finish before running the state machine
        }

        // ------------------------------------------------------------------
        // When IDLE: accept bumper (colour-targeted) or X (current slot).
        // ------------------------------------------------------------------
        if (shootSubState == ShootSubState.IDLE) {
            if (lbEdge) {
                pendingShootColor = "G";
                startShot();
            } else if (rbEdge) {
                pendingShootColor = "P";
                startShot();
            }
        }

        // --- Shooting sub-state machine ---
        switch (shootSubState) {

            case IDLE:
                break;

            case MOVE_TO_SLOT:
                moveToSlot(targetSlot);
                sorterMoved     = false;
                rotateStartTime = currentTime;
                shootSubState   = ShootSubState.ROTATE_SORTER;
                break;

            case ROTATE_SORTER:
                if (currentTime - rotateStartTime < ROTATE_GUARD_SEC) break;

                if (!sorterMoved && sorter.atTarget()) {
                    moveToSlot(targetSlot);
                    sorterMoved = true;
                } else if (sorterMoved && sorter.atTarget()) {
                    transferInProgress = true;
                    shootSubState      = ShootSubState.TRANSFER;
                }
                break;

            case TRANSFER:
                // Fire when sorter is in place AND shooter is at speed.
                // Timeout after 3s in case the shooter never reaches target velocity
                // (low battery) — fire anyway rather than hanging forever.
                boolean shooterReady = shooter.atTargetVel();
                boolean transferTimedOut = (currentTime - rotateStartTime) > 3.0;
                if (transferInProgress && sorter.atTarget() && (shooterReady || transferTimedOut)) {
                    transferStartTime = currentTime;
                    hw.sorterTransfer.setPosition(RobotHardware.transferPush);
                    shootSubState = ShootSubState.RESET;
                }
                break;

            case RESET:
                // Phase 1 — retract servo after 0.5s
                if (transferInProgress && currentTime - transferStartTime > 0.23) {
                    hw.sorterTransfer.setPosition(RobotHardware.transferIdle);
                    removeArtifact(currentSlot);
                    artifactCount--;
                    transferInProgress = false;
                    sorterMoved        = false;
                    pendingShootColor  = null;
                    transferStartTime  = currentTime; // reuse timer for phase 2
                }
                // Phase 2 — wait for servo to physically clear the sorter before moving
                if (!transferInProgress && currentTime - transferStartTime > 0.25) {
                    if (artifactCount <= 0) {
                        artifactCount = 0;
                        autoFire      = false;
                        enterManualState();
                    } else if (autoFire) {
                        startNextAutoShot();
                    } else {
                        shootSubState = ShootSubState.IDLE;
                    }
                }
                break;
        }
    }

    // Picks the next available slot and fires — used by auto fire sequence.
    private void startNextAutoShot() {
        int slot = findAnySlot();
        if (slot == -1) {
            autoFire = false;
            enterManualState();
            return;
        }
        targetSlot        = slot;
        pendingShootColor = storedArtifacts[slot];
        shootSubState     = ShootSubState.MOVE_TO_SLOT;
    }

    private void startShot() {
        int slot = findSlotByColor(pendingShootColor);
        if (slot == -1) slot = findAnySlot();
        if (slot == -1) {
            pendingShootColor = null;
            return;
        }
        targetSlot    = slot;
        shootSubState = ShootSubState.MOVE_TO_SLOT;
    }

    // =========================================================================
    // MANUAL STATE
    // =========================================================================
    private void updateManual(boolean ltEdge, boolean rtEdge,
                              boolean servoEdge, double currentTime) {
        // dpad_right -> enterIntakeState() handled globally in update().

        long now = System.currentTimeMillis();

        if (!manualTransferActive) {
            // Servo is idle — allow sorter rotation
            if (now - lastSlotSwitchTime > SLOT_DEBOUNCE_MS) {
                if (ltEdge) {
                    sorter.moveDegrees(-RobotHardware.DEG_PER_SLOT);
                    lastSlotSwitchTime = now;
                } else if (rtEdge) {
                    sorter.moveDegrees(RobotHardware.DEG_PER_SLOT);
                    lastSlotSwitchTime = now;
                }
            }

            // Allow fire only when sorter has settled
            if (servoEdge && sorter.atTarget()) {
                manualTransferActive = true;
                manualTransferStart  = currentTime;
                hw.sorterTransfer.setPosition(RobotHardware.transferPush);
            }
        } else {
            // Servo is active — retract after timeout, then unlock rotation
            if (currentTime - manualTransferStart > MANUAL_TRANSFER_SEC) {
                hw.sorterTransfer.setPosition(RobotHardware.transferIdle);
                manualTransferActive = false;
            }
            // Sorter rotation intentionally blocked while servo is out
        }
    }

    // =========================================================================
    // STATE TRANSITION HELPERS
    // =========================================================================
    private void enterShootingState() {
        robotState           = RobotState.SHOOTING;
        shootSubState        = ShootSubState.IDLE;
        sorterMoved          = false;
        transferInProgress   = false;
        pendingShootColor    = null;
        manualTransferActive = false;
        autoFire             = false;
        intake.intakeOn      = false;
        shooter.setTargetVelRPM(rpm);
    }

    private void enterManualState() {
        robotState           = RobotState.MANUAL;
        shootSubState        = ShootSubState.IDLE;
        manualTransferActive = false;
        // Shooter intentionally stays spinning
    }

    private void enterIntakeState() {
        shooter.setTargetVelRPM(0);
        intake.intakeOn      = false;
        manualTransferActive = false;
        hw.sorterTransfer.setPosition(RobotHardware.transferIdle);

        if (offSetApplied) {
            sorter.moveDegrees(60);
        }

        transferInProgress = false;
        sorterMoved        = false;
        pendingShootColor  = null;
        inspectSlotIndex   = 0;
        intakeSubState     = IntakeSubState.IDLE;
        shootSubState      = ShootSubState.IDLE;
        robotState         = RobotState.INTAKE;

        resetSlot();
    }

    // =========================================================================
    // COLOUR DETECTION (sensor-based)
    // =========================================================================
    private void detect() {
        double d1 = hw.colorSensor.getDistance(DistanceUnit.MM);
        double d2 = hw.colorSensor2.getDistance(DistanceUnit.MM);

        boolean detectedNow = (d1 < 50 || d2 < 50);
        boolean risingEdge  = detectedNow && !lastDetected;

        long now = System.currentTimeMillis();

        if (!artifactPresent && risingEdge && now - lastDetectionTime > 300) {
            artifactPresent = true;

            int maxGreen = Math.max(hw.colorSensor.green(), hw.colorSensor2.green());
            int maxBlue  = Math.max(hw.colorSensor.blue(),  hw.colorSensor2.blue());
            String color = (maxGreen > maxBlue) ? "G" : "P";

            if (artifactCount < storedArtifacts.length) {
                storedArtifacts[nextSlotIndex] = color;
                nextSlotIndex = (nextSlotIndex + 1) % 3;
            }
            lastDetectionTime = now;
        }
        lastDetected = detectedNow;
    }

    private void manualDetect(String color) {
        if (artifactCount >= storedArtifacts.length) return;
        storedArtifacts[nextSlotIndex] = color;
        nextSlotIndex   = (nextSlotIndex + 1) % 3;
        artifactPresent = true;
    }

    // =========================================================================
    // SLOT / ARTIFACT MANAGEMENT
    // =========================================================================
    private void moveToSlot(int slot) {
        if (storedArtifacts[slot] == null) return;

        if (!offSetApplied) {
            double deg;
            switch (slot) {
                case 0:  deg = -60;  break;
                case 1:  deg =  60;  break;
                case 2:  deg = 180;  break;
                default: deg =   0;
            }
            sorter.moveDegrees(deg);
            offSetApplied = true;
            currentSlot   = slot;
            return;
        }

        int delta = (slot - currentSlot + 3) % 3;
        int step  = (delta == 0) ? 0 : (delta == 1) ? 1 : -1;
        double deg = step * RobotHardware.DEG_PER_SLOT;
        if (deg != 0.0) sorter.moveDegrees(deg);
        currentSlot = slot;
    }

    private void rotateToPhysicalSlot(int slot) {
        if (!offSetApplied) {
            double deg;
            switch (slot) {
                case 0:  deg = -60;  break;
                case 1:  deg =  60;  break;
                case 2:  deg = 180;  break;
                default: deg =   0;
            }
            sorter.moveDegrees(deg);
            offSetApplied = true;
            currentSlot   = slot;
            return;
        }

        int delta = (slot - currentSlot + 3) % 3;
        int step  = (delta == 0) ? 0 : (delta == 1) ? 1 : -1;
        double deg = step * RobotHardware.DEG_PER_SLOT;
        if (deg != 0.0) sorter.moveDegrees(deg);
        currentSlot = slot;
    }

    private int findSlotByColor(String color) {
        for (int i = 0; i < storedArtifacts.length; i++) {
            if (Objects.equals(storedArtifacts[i], color)) return i;
        }
        return -1;
    }

    private int findNextSlotByColor(String color, int startIndex) {
        for (int i = 0; i < storedArtifacts.length; i++) {
            int idx = (startIndex + i) % storedArtifacts.length;
            if (Objects.equals(storedArtifacts[idx], color)) return idx;
        }
        return -1;
    }

    private int findAnySlot() {
        for (int i = 0; i < storedArtifacts.length; i++) {
            if (storedArtifacts[i] != null) return i;
        }
        return -1;
    }

    private void removeArtifact(int slot) {
        if (slot < 0 || slot >= storedArtifacts.length) return;
        if (storedArtifacts[slot] != null) {
            storedArtifacts[slot] = null;
            motifProgress = (motifProgress + 1) % motif.length;
        }
    }

    public void resetSlot() {
        currentSlot   = 0;
        artifactCount = 0;
        nextSlotIndex = 0;
        motifProgress = 0;
        offSetApplied = false;
        Arrays.fill(storedArtifacts, null);
    }

    public boolean isActivelyShooting() {
        return robotState == RobotState.SHOOTING
                && shootSubState != ShootSubState.IDLE;
    }
}