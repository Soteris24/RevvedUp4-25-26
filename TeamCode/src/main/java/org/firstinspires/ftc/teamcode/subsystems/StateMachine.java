package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class StateMachine {

    private final RobotHardware hw;
    private final Sorter sorter;
    private final Shooter2 shooter;
    private final ArtifactSorter artifactSorter;
    private final Intake intake;

    private final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    private final boolean telemetryOn;

    // Shooting state
    public enum ShootState { IDLE, MOVE_TO_SLOT, ROTATE_SORTER, TRANSFER, RESET, RETURN }
    private ShootState shootState = ShootState.IDLE;

    // Intake state
    public enum IntakeState { IDLE, INTAKE, WAIT, RETURN, FULL }
    private IntakeState intakeState = IntakeState.IDLE;
    // In StateMachine.java
    public ShootState getShootState() {
        return shootState;
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }

    // Timers
    private long detectTime = 0;
    private double transferStartTime = 0;
    private boolean transferInProgress = false;
    private boolean sorterMoved = false;

    // Shooter RPM (autonomous selection)
    private double rpm = 2100;
    public enum AutoColor { RED, BLUE }
    private AutoColor autoColor = AutoColor.RED;

    public StateMachine(RobotHardware hw, Sorter sorter, Shooter2 shooter,
                        ArtifactSorter artifactSorter, Intake intake,
                        boolean telemetryOn, AutoColor autoColor) {
        this.hw = hw;
        this.sorter = sorter;
        this.shooter = shooter;
        this.artifactSorter = artifactSorter;
        this.intake = intake;
        this.telemetryOn = telemetryOn;
        this.autoColor = autoColor;

        rpm = (autoColor == AutoColor.RED) ? 2700 : 2100;
    }

    // -----------------------------
    // MAIN SHOOTING LOOP
    // -----------------------------
    public void updateShooting(boolean farButton, boolean closeButton, double currentTime) {
        if (artifactSorter.artifactCount > 0) {
            if (artifactSorter.motifSortingEnabled) {
                artifactSorter.motifSorting();
            } else {
                artifactSorter.pickNextArtifact();
            }
        }

        // Update RPM based on autonomous selection / buttons
        if (farButton) rpm = 2700;
        if (closeButton) rpm = 2100;
        if (artifactSorter.artifactCount > 0) shooter.setTargetVelRPM(rpm);

        switch (shootState) {
            case IDLE:
                if ((farButton || closeButton) && artifactSorter.artifactCount > 0) {
                    artifactSorter.artifactPresent = false;
                    shootState = ShootState.MOVE_TO_SLOT;
                }
                break;

            case MOVE_TO_SLOT:
                artifactSorter.moveToSlot(artifactSorter.targetSlot);
                shooter.setTargetVelRPM(rpm);
                sorterMoved = false;
                shootState = ShootState.ROTATE_SORTER;
                break;

            case ROTATE_SORTER:
                if (!sorterMoved && sorter.atTarget()) {
                    artifactSorter.moveToSlot(artifactSorter.targetSlot);
                    sorterMoved = true;
                }
                if (sorterMoved && sorter.atTarget()) {
                    transferInProgress = true;
                    shootState = ShootState.TRANSFER;
                }
                break;

            case TRANSFER:
                if (transferInProgress && sorter.atTarget() && shooter.atTargetVel()) {
                    transferStartTime = currentTime;
                    hw.sorterTransfer.setPosition(RobotHardware.transferPush);
                    shootState = ShootState.RESET;
                }
                break;

            case RESET:
                if (transferInProgress && currentTime - transferStartTime > 0.23) {
                    hw.sorterTransfer.setPosition(RobotHardware.transferIdle);
                    transferInProgress = false;
                    artifactSorter.removeArtifact(artifactSorter.currentSlot);
                    artifactSorter.artifactCount--;
                    transferStartTime = currentTime;
                }
                if (!transferInProgress && currentTime - transferStartTime > 0.25) {
                    if (artifactSorter.artifactCount < 1) {
                        sorterMoved = false;
                        shootState = ShootState.RETURN;
                    } else {
                        sorterMoved = false;
                        shootState = ShootState.MOVE_TO_SLOT;
                    }
                }
                break;

            case RETURN:
                if (!sorterMoved) {
                    sorter.moveDegrees(60);
                    sorterMoved = true;
                    artifactSorter.artifactPresent = false;
                    artifactSorter.offSetApplied = false;
                    artifactSorter.resetSlot();
                }
                if (sorterMoved && sorter.atTarget()) {
                    shootState = ShootState.IDLE;
                }
                break;
        }

        if (telemetryOn) {
            panelsTelemetry.getTelemetry().addData("ShootState", shootState);
            panelsTelemetry.getTelemetry().addData("RPM", rpm);
            panelsTelemetry.getTelemetry().addData("TargetSlot", artifactSorter.targetSlot);
            panelsTelemetry.getTelemetry().addData("Artifacts", artifactSorter.artifactCount);
        }
    }

    // -----------------------------
    // MAIN INTAKE LOOP
    // -----------------------------
    public void updateIntake(boolean intakeButton, boolean reverseButton, double currentTime,
                             boolean buttonG, boolean buttonP) {
        intake.intake(intakeButton, reverseButton, currentTime);

        if (shootState == ShootState.IDLE) artifactSorter.detect();

        shooter.setTargetVelRPM(intake.intakeOn ? 1000 : (shootState == ShootState.IDLE ? 0 : rpm));

        long sorterWaitTime = 0;

        switch (intakeState) {
            case IDLE:
                if (intake.intakeOn) {
                    artifactSorter.artifactPresent = false;
                    intakeState = IntakeState.INTAKE;
                }
                break;

            case INTAKE:
                if (artifactSorter.artifactPresent) {
                    detectTime = System.currentTimeMillis();
                    sorterMoved = false;
                    intakeState = IntakeState.WAIT;
                }
                break;

            case WAIT:
                if (!sorterMoved && System.currentTimeMillis() - detectTime >= sorterWaitTime
                        && artifactSorter.artifactPresent) {
                    if (artifactSorter.artifactCount < 2) {
                        sorter.moveDegrees(120);
                        sorterMoved = true;
                        intakeState = IntakeState.RETURN;
                    } else {
                        intakeState = IntakeState.FULL;
                    }
                }
                break;

            case RETURN:
                if (sorter.atTarget()) {
                    if (artifactSorter.artifactCount >= 3) {
                        artifactSorter.artifactPresent = true;
                        intakeState = IntakeState.FULL;
                    } else {
                        artifactSorter.artifactPresent = false;
                        artifactSorter.artifactCount++;
                        intakeState = IntakeState.INTAKE;
                    }
                }
                break;

            case FULL:
                artifactSorter.artifactCount = 3;
                artifactSorter.moveToSlot(artifactSorter.targetSlot);
                artifactSorter.artifactPresent = true;
                intake.intakeOn = false;
                if (sorter.atTarget()) intakeState = IntakeState.IDLE;
                break;
        }

        if (telemetryOn) {
            panelsTelemetry.getTelemetry().addData("IntakeState", intakeState);
            panelsTelemetry.getTelemetry().addData("IntakeOn", intake.intakeOn);
            panelsTelemetry.getTelemetry().addData("ArtifactPresent", artifactSorter.artifactPresent);
            panelsTelemetry.getTelemetry().addData("StoredArtifacts", artifactSorter.storedArtifacts);
            panelsTelemetry.getTelemetry().addData("ArtifactCount", artifactSorter.artifactCount);
        }
    }
}