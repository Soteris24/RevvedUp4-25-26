package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.RobotHardware;


public class StateMachine {
    RobotHardware hw;
    private final Sorter sorter;
    private final Shooter2 shooter;
    private final ArtifactSorter artifactSorter;
    private final Intake intake;

    private final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

    private final boolean telemetryOn;

    private long detectTime;

    private boolean sorterMoved = false;
    private boolean transferInProgress = false;
    private double transferStartTime = 0;
    double rpm = 1200;

    public StateMachine(RobotHardware hw, Sorter sorter, Shooter2 shooter, ArtifactSorter artifactSorter, Intake intake, boolean telemetryOn) {
        this.hw = hw;
        this.sorter = sorter;
        this.shooter = shooter;
        this.artifactSorter = artifactSorter;
        this.intake = intake;
        this.telemetryOn = telemetryOn;
    }

    public enum ShootState{
        IDLE, MOVE_TO_SLOT, ROTATE_SORTER, TRANSFER, RESET, RETURN
    }
    public ShootState shootState = ShootState.IDLE;

    public enum IntakeState {
        IDLE, INTAKE, WAIT, RETURN, FULL
    }
     public IntakeState intakeState = IntakeState.IDLE;

    public void updateShooting(boolean farButton, boolean closeButton, double currentTime) {
        if (artifactSorter.artifactCount > 0) {
            if (artifactSorter.motifSortingEnabled) {
                artifactSorter.motifSorting();
            } else {
                artifactSorter.pickNextArtifact();
            }
        }
        if (farButton) {
            rpm = 2700;
        } else if (closeButton) {
            rpm = 2100;
        }
        if (artifactSorter.artifactCount == 3) {
            shooter.setTargetVelRPM(rpm);
        }

        switch (shootState) {
            case IDLE:
                if ((closeButton || farButton) && artifactSorter.artifactCount > 0) {
                    artifactSorter.artifactPresent = false;
                    setShootState(ShootState.MOVE_TO_SLOT);                }
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
                if (transferInProgress  && sorter.atTarget()&& shooter.atTargetVel()) { //&& shooter.atTargetVel()
                    transferStartTime = currentTime;
                    hw.sorterTransfer.setPosition(RobotHardware.transferPush);
                    shootState = ShootState.RESET;
                }
                break;
            case RESET:
                if (transferInProgress && currentTime - transferStartTime > 0.5) {
                    hw.sorterTransfer.setPosition(RobotHardware.transferIdle);
                    transferStartTime = currentTime;
                    transferInProgress = false;
                    artifactSorter.removeArtifact(artifactSorter.currentSlot);
                    artifactSorter.artifactCount--;
                }
                if (!transferInProgress && currentTime - transferStartTime > 0.5) {
                    if (artifactSorter.artifactCount < 1) {
                        sorterMoved = false;
                        setShootState(ShootState.RETURN);
                    }
                    else {
                        sorterMoved = false;
                        setShootState(ShootState.MOVE_TO_SLOT);
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
                    setShootState(ShootState.IDLE);
                }
        }

        if (telemetryOn) {
            panelsTelemetry.getTelemetry().addData("ShootState", shootState);
            panelsTelemetry.getTelemetry().addData("TargetSlot", artifactSorter.targetSlot);
            panelsTelemetry.getTelemetry().addData("Artifacts", artifactSorter.artifactCount);
        }
    }

    public void updateIntake(boolean button, boolean reverseButton, double currentTime, boolean buttonG, boolean buttonP){
        intake.intake(button, reverseButton, currentTime);

        if (shootState == ShootState.IDLE) {
            artifactSorter.detect2(buttonP, buttonG);
        }

        if (intake.intakeOn) {
            shooter.setTargetVelRPM(1000);
        } else if (shootState == ShootState.IDLE){
            shooter.setTargetVelRPM(0);
        }

        long sorterWaitTime = 0;

        switch (intakeState) {
            case IDLE:
                if (intake.intakeOn) {
                    artifactSorter.artifactPresent = false;
                    setIntakeState(IntakeState.INTAKE);
                }
                break;
            case INTAKE:
                if(artifactSorter.artifactPresent) {
                    detectTime = System.currentTimeMillis();
                    sorterMoved = false;
                    setIntakeState(IntakeState.WAIT);
                }
                break;
            case WAIT:
                if (!sorterMoved
                        && Math.abs(System.currentTimeMillis() - detectTime) >= sorterWaitTime
                        && artifactSorter.artifactPresent) {
                    if (artifactSorter.artifactCount < 2) {
                        sorter.moveDegrees(120);
                        sorterMoved = true;
                        setIntakeState(IntakeState.RETURN);
                    } else {
                        setIntakeState(IntakeState.FULL);
                    }
                }
                break;
            case RETURN:
                if (sorter.atTarget()) {
                    if (artifactSorter.artifactCount >= 3) {
                        artifactSorter.artifactPresent = true;
                        setIntakeState(IntakeState.FULL);
                    } else {
                        artifactSorter.artifactPresent = false;
                        artifactSorter.artifactCount++;
                        setIntakeState(IntakeState.INTAKE);
                    }
                }
                break;
            case FULL:
                artifactSorter.artifactCount = 3;
                artifactSorter.moveToSlot(artifactSorter.targetSlot);
                artifactSorter.artifactPresent = true;
                intake.intakeOn = false;
                if (sorter.atTarget()) {
                    setIntakeState(IntakeState.IDLE);
                }
                break;
        }
        if (telemetryOn) {
            panelsTelemetry.getTelemetry().addData("IntakeState", intakeState);
            panelsTelemetry.getTelemetry().addData("IntakeOn", intake.intakeOn);
            panelsTelemetry.getTelemetry().addData("Artifact Present", artifactSorter.artifactPresent);
            panelsTelemetry.getTelemetry().addData("Stored Artifacts", artifactSorter.storedArtifacts);
            panelsTelemetry.getTelemetry().addData("Artifact Count", artifactSorter.artifactCount);
        }
    }

    void setShootState (ShootState state) {
        shootState = state;
    }
    void setIntakeState (IntakeState state) {
        intakeState = state;
    }
}
