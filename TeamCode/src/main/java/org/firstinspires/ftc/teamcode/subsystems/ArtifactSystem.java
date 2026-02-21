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
 *   X                  - reverse intake for 0.5 secs
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
 *   dpad_up            — increase RPM
 *   dpad_down          — decrease RPM
 *   dpad_right         — exit to INTAKE
 *
 * MANUAL STATE:
 *   left_trigger       — rotate sorter one step left  (free rotation)
 *   right_trigger      — rotate sorter one step right (free rotation)
 *   Y                  - fire current slot
 *   dpad_right         — exit to INTAKE
 *
 * ======================================================================
 */
public class ArtifactSystem {

    // =========================================================================
    // DEPENDENCIES
    // =========================================================================
    private final RobotHardware hw;
    private final Sorter sorter;
    private final Shooter2 shooter;
    private final Intake intake;
    private final Telemetry telemetry;
    private final PanelsTelemetry panelsTel = PanelsTelemetry.INSTANCE;
    private final boolean telemetryOn;

    // =========================================================================
    // PUBLIC STATE ENUMS
    // =========================================================================
    public enum RobotState { INTAKE, SHOOTING, MANUAL }
    public enum ShootSubState { IDLE, MOVE_TO_SLOT, ROTATE_SORTER, TRANSFER, RESET }
    public enum IntakeSubState { IDLE, INTAKE, WAIT, RETURN, FULL }

    public RobotState robotState = RobotState.INTAKE;
    public ShootSubState shootSubState = ShootSubState.IDLE;
    public IntakeSubState intakeSubState = IntakeSubState.IDLE;

    // =========================================================================
    // ARTIFACT STORAGE
    // =========================================================================
    public String[] storedArtifacts = new String[3];
    public int artifactCount = 0;
    public int targetSlot = 0;
    public int currentSlot = 0;
    public int nextSlotIndex = 0;
    public boolean offSetApplied   = false;
    public boolean artifactPresent = false;

    // Motif sorting (optional)
    public boolean  motifSortingEnabled = false;
    public String[] motif = {"G", "P", "P"};
    private int motifProgress = 0;

    // =========================================================================
    // SHOOTING INTERNALS
    // =========================================================================
    public double rpm = SHORT_RPM;
    public static final double SHORT_RPM = 2100;
    public static final double LONG_RPM  = 2700;

    private String pendingShootColor = null;
    private boolean transferInProgress = false;
    private double transferStartTime = 0;
    private boolean sorterMoved = false;
    private boolean manualTransferActive = false;

    // Guard timer — prevents ROTATE_SORTER from seeing atTarget() before
    // the motor has physically started moving.
    private double rotateStartTime         = 0;
    private static final double ROTATE_GUARD_SEC = 0.15;

    // =========================================================================
    // COLOUR DETECTION INTERNALS
    // =========================================================================
    private boolean lastDetected = false;
    private long lastDetectionTime = 0;

    // =========================================================================
    // BUTTON EDGE-DETECTION HISTORY
    // =========================================================================
    private boolean lastDpadUp, lastDpadDown, lastDpadLeft, lastDpadRight;
    private boolean lastLeftTrigger, lastRightTrigger;
    private boolean lastLeftBumper, lastRightBumper;
    private boolean lastIntakeToggle;
    private boolean lastServoTransfer;
    private boolean lastintakeReverse;


    // =========================================================================
    // DEBOUNCE
    // =========================================================================
    private long lastSlotSwitchTime = 0;
    private static final long SLOT_DEBOUNCE_MS = 300;

    // Tracks which slot index the "inspect/fix" cycling is currently looking at
    private int inspectSlotIndex = 0;

    // =========================================================================
    // CONSTRUCTOR
    // =========================================================================
    public ArtifactSystem(RobotHardware hw, Telemetry telemetry,
                          Sorter sorter, Shooter2 shooter, Intake intake,
                          boolean telemetryOn) {
        this.hw = hw;
        this.telemetry = telemetry;
        this.sorter = sorter;
        this.shooter = shooter;
        this.intake = intake;
        this.telemetryOn = telemetryOn;
    }

    // =========================================================================
    // MAIN UPDATE — call every loop iteration
    // =========================================================================
    /**
     * @param dpadUp         gamepad2.dpad_up
     * @param dpadDown       gamepad2.dpad_down
     * @param dpadLeft       gamepad2.dpad_left
     * @param dpadRight      gamepad2.dpad_right
     * @param leftTrigger    gamepad2.left_trigger  > 0.5
     * @param rightTrigger   gamepad2.right_trigger > 0.5
     * @param leftBumper     gamepad2.left_bumper
     * @param rightBumper    gamepad2.right_bumper
     * @param intakeToggle   gamepad2.y
     * @param servoTranfer   gamepad2.x
     * @param currentTime    opmode getRuntime()
     */
    public void update(boolean dpadUp,       boolean dpadDown,
                       boolean dpadLeft,     boolean dpadRight,
                       boolean leftTrigger,  boolean rightTrigger,
                       boolean leftBumper,   boolean rightBumper,
                       boolean intakeToggle, boolean servoTranfer,
                       boolean intakereverse,
                       double  currentTime) {

        // --- Rising edges ---
        boolean upEdge     = dpadUp     && !lastDpadUp;
        boolean downEdge   = dpadDown   && !lastDpadDown;
        boolean leftEdge   = dpadLeft   && !lastDpadLeft;
        boolean rightEdge  = dpadRight  && !lastDpadRight;
        boolean ltEdge     = leftTrigger  && !lastLeftTrigger;
        boolean rtEdge     = rightTrigger && !lastRightTrigger;
        boolean lbEdge     = leftBumper   && !lastLeftBumper;
        boolean rbEdge     = rightBumper  && !lastRightBumper;
        boolean intakeEdge = intakeToggle && !lastIntakeToggle;
        boolean servoedge = servoTranfer && !lastServoTransfer;
        boolean intakerevEgde = intakereverse && !lastintakeReverse;

        // --- Global state transitions ---
        // dpad_right → always exit shoot / manual back to intake
        if (rightEdge && (robotState == RobotState.SHOOTING || robotState == RobotState.MANUAL)) {
            enterIntakeState();
        }
        // dpad_left → enter shoot (short) from intake
        else if (leftEdge && robotState == RobotState.INTAKE) {
            rpm = SHORT_RPM;
            enterShootingState();
        }
        // dpad_up → enter shoot (long) from intake; or SET long RPM if already shooting
        else if (upEdge) {
            if (robotState == RobotState.INTAKE) {
                rpm = LONG_RPM;
                enterShootingState();
            } else if (robotState == RobotState.SHOOTING) {
                rpm = LONG_RPM;
                shooter.setTargetVelRPM(rpm);
            }
        }
        // dpad_down → SET short RPM in shoot state
        else if (downEdge && robotState == RobotState.SHOOTING) {
            rpm = SHORT_RPM;
            shooter.setTargetVelRPM(rpm);
        }

        // --- Per-state update ---
        switch (robotState) {
            case INTAKE:
                updateIntake(lbEdge, rbEdge, ltEdge, rtEdge, intakeEdge, currentTime, intakerevEgde);
                break;
            case SHOOTING:
                updateShooting(lbEdge, rbEdge, currentTime);
                break;
            case MANUAL:
                updateManual(ltEdge, rtEdge, servoedge, currentTime);
                break;
        }

        // --- Save button history ---
        lastDpadUp    = dpadUp;    lastDpadDown  = dpadDown;
        lastDpadLeft  = dpadLeft;  lastDpadRight = dpadRight;
        lastLeftTrigger  = leftTrigger;  lastRightTrigger = rightTrigger;
        lastLeftBumper   = leftBumper;   lastRightBumper  = rightBumper;
        lastIntakeToggle = intakeToggle;
        lastServoTransfer =servoTranfer;

        // --- Telemetry ---
        if (telemetryOn) {
            panelsTel.getTelemetry().addData("RobotState",     robotState);
            panelsTel.getTelemetry().addData("ShootSubState",  shootSubState);
            panelsTel.getTelemetry().addData("IntakeSubState", intakeSubState);
            panelsTel.getTelemetry().addData("ArtifactCount",  artifactCount);
            panelsTel.getTelemetry().addData("Artifacts",      Arrays.toString(storedArtifacts));
            panelsTel.getTelemetry().addData("CurrentSlot",    currentSlot);
            panelsTel.getTelemetry().addData("TargetSlot",     targetSlot);
            panelsTel.getTelemetry().addData("RPM Target",     rpm);
            panelsTel.getTelemetry().addData("IntakeOn",       intake.intakeOn);
        }
    }

    // =========================================================================
    // INTAKE STATE
    // =========================================================================
    private void updateIntake(boolean lbEdge, boolean rbEdge,
                              boolean ltEdge,  boolean rtEdge,
                              boolean intakeEdge,
                              double currentTime, boolean intakerevedge) {

        // Y toggles intake on/off
        if (intakeEdge) {
            intake.intakeOn = !intake.intakeOn;
            if (intake.intakeOn) {
                artifactPresent = false;
                intakeSubState  = IntakeSubState.INTAKE;
            } else {
                intakeSubState = IntakeSubState.IDLE;
            }
        }

        // drive the intake motor every tick based on intake.intakeOn
        // (intakeOn is a flag; the Intake subsystem needs to be called to apply power)
        intake.intake(false, intakerevedge, currentTime);

        // Run colour detection only while actively intaking
        if (intakeSubState == IntakeSubState.INTAKE) {
            detect();
        }

        // ------------------------------------------------------------------
        // Bumpers = manual colour assignment.
        // Left bumper  = mark next free slot as GREEN.
        // Right bumper = mark next free slot as PURPLE.
        // We set manualOverride so the sub-state machine doesn't also fire
        // a moveDegrees() on the same tick.
        // ------------------------------------------------------------------
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

        // ------------------------------------------------------------------
        // Triggers = inspect/fix mode.
        // Cycles the sorter through slots of the requested colour so the
        // driver can verify or re-collect a missed/wrong ball.
        // Left trigger  = find next GREEN  slot and rotate to it.
        // Right trigger = find next PURPLE slot and rotate to it.
        // ------------------------------------------------------------------
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

        if (manualOverride) return; // skip sub-state machine this tick

        // --- Intake auto-rotation sub-state machine ---
        switch (intakeSubState) {
            case IDLE:
                // Waiting for intake toggle — nothing to do.
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

        // Keep flywheel warm during intake
        shooter.setTargetVelRPM(intake.intakeOn ? 1000 : 0);
    }

    // =========================================================================
    // SHOOT STATE
    // =========================================================================
    private void updateShooting(boolean lbEdge, boolean rbEdge, double currentTime) {

        shooter.setTargetVelRPM(rpm);

        // Accept a new bumper press only while idle (between shots)
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
                // Guard: don't check atTarget() until the motor has had time to start moving
                if (currentTime - rotateStartTime < ROTATE_GUARD_SEC) break;

                if (!sorterMoved && sorter.atTarget()) {
                    moveToSlot(targetSlot); // confirm position
                    sorterMoved = true;
                } else if (sorterMoved && sorter.atTarget()) {
                    transferInProgress = true;
                    shootSubState      = ShootSubState.TRANSFER;
                }
                break;

            case TRANSFER:
                if (transferInProgress && sorter.atTarget() && shooter.atTargetVel()) {
                    transferStartTime = currentTime;
                    hw.sorterTransfer.setPosition(RobotHardware.transferPush);
                    shootSubState = ShootSubState.RESET;
                }
                break;

            case RESET:
                // Retract pusher after 0.5 s
                if (transferInProgress && currentTime - transferStartTime > 0.5) {
                    hw.sorterTransfer.setPosition(RobotHardware.transferIdle);
                    transferStartTime  = currentTime;
                    transferInProgress = false;
                    removeArtifact(currentSlot);
                    artifactCount--;
                }
                // 0.5 s later decide next step
                if (!transferInProgress && currentTime - transferStartTime > 0.5) {
                    sorterMoved       = false;
                    pendingShootColor = null;

                    if (artifactCount <= 0) {
                        artifactCount = 0;
                        enterManualState();
                    } else {
                        shootSubState = ShootSubState.IDLE; // wait for next bumper press
                    }
                }
                break;
        }
    }

    private void startShot() {
        int slot = findSlotByColor(pendingShootColor);
        if (slot == -1) slot = findAnySlot(); // fallback
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
    private void updateManual(boolean ltEdge, boolean rtEdge, boolean servoPressed, double currentTime) {
        // Shooter stays spinning at last RPM.
        // Triggers rotate the sorter freely (not snapped to the 3 intake slots).
        // dpad_right -> enterIntakeState() handled globally in update().

        long now = System.currentTimeMillis();
        if (now - lastSlotSwitchTime > SLOT_DEBOUNCE_MS) {
            if (ltEdge) {
                sorter.moveDegrees(-RobotHardware.DEG_PER_SLOT);
                lastSlotSwitchTime = now;
            } else if (rtEdge) {
                sorter.moveDegrees(RobotHardware.DEG_PER_SLOT);
                lastSlotSwitchTime = now;
            }
        }
        if (servoPressed && !manualTransferActive) {
            manualTransferActive = true;
            transferStartTime = currentTime;
            hw.sorterTransfer.setPosition(RobotHardware.transferPush);
        }

        // Retract after 0.4 seconds
        if (manualTransferActive && currentTime - transferStartTime > 0.4) {
            hw.sorterTransfer.setPosition(RobotHardware.transferIdle);
            manualTransferActive = false;
        }
    }

    // =========================================================================
    // STATE TRANSITION HELPERS
    // =========================================================================
    private void enterShootingState() {
        robotState         = RobotState.SHOOTING;
        shootSubState      = ShootSubState.IDLE;
        sorterMoved        = false;
        transferInProgress = false;
        pendingShootColor  = null;
        intake.intakeOn    = false;
        shooter.setTargetVelRPM(rpm);
    }

    private void enterManualState() {
        robotState    = RobotState.MANUAL;
        shootSubState = ShootSubState.IDLE;
        // Shooter intentionally stays spinning
    }

    private void enterIntakeState() {
        shooter.setTargetVelRPM(0);
        intake.intakeOn = false;

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

        boolean detectedNow = (d1 < 25 || d2 < 25);
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

    // Manual detect — called by bumper presses in intake state
    private void manualDetect(String color) {
        if (artifactCount >= storedArtifacts.length) return;
        storedArtifacts[nextSlotIndex] = color;
        nextSlotIndex   = (nextSlotIndex + 1) % 3;
        artifactPresent = true;
    }

    // =========================================================================
    // SLOT / ARTIFACT MANAGEMENT
    // =========================================================================

    /**
     * Rotate the sorter to the physical position of a given slot index,
     * accounting for whether an offset has already been applied.
     * Used by both shooting and inspect/fix in intake.
     */
    private void moveToSlot(int slot) {
        if (storedArtifacts[slot] == null) return;

        if (!offSetApplied) {
            double deg;
            switch (slot) {
                case 0:  deg = -60;  break;
                case 1:  deg =  60;  break;
                case 2:  deg =  180; break;
                default: deg =  0;
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

    /**
     * Rotate to a slot regardless of whether it has an artifact (used for
     * inspect/fix cycles where the slot might be empty).
     */
    private void rotateToPhysicalSlot(int slot) {
        if (!offSetApplied) {
            double deg;
            switch (slot) {
                case 0:  deg = -60;  break;
                case 1:  deg =  60;  break;
                case 2:  deg =  180; break;
                default: deg =  0;
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

    /**
     * Find the next slot of a given color starting from startIndex (wraps around).
     * Used by inspect/fix trigger cycling.
     */
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

    /** DriverOp uses this to slow drivetrain rotation while a shot is in progress. */
    public boolean isActivelyShooting() {
        return robotState == RobotState.SHOOTING
                && shootSubState != ShootSubState.IDLE;
    }
}