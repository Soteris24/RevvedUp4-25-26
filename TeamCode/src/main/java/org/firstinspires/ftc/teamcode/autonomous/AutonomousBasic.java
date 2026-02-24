package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArtifactSorter;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter2;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;
import org.firstinspires.ftc.teamcode.subsystems.StateMachine;
@Disabled
@Autonomous(name = "Autonomous Basic")
public class AutonomousBasic extends LinearOpMode {

    // Hardware and subsystems
    RobotHardware hw;
    Drivetrain drivetrain;
    Sorter sorter;
    Shooter2 shooter;
    Intake intake;
    ArtifactSorter artifactSorter;
    StateMachine stateMachine;

    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime stateTimer = new ElapsedTime();

    // Tunable parameters
    double BACKUP_POWER = -0.3;
    double BACKUP_TIME = 1.0;
    double SHOOTER_RPM = 2100;

    // State machine for autonomous
    enum AutoState {
        INIT,
        BACKUP,
        SPIN_UP_SHOOTER,
        SHOOT_BALLS,
        RETURN_HOME,
        COMPLETE
    }

    AutoState currentState = AutoState.INIT;

    @Override
    public void runOpMode() {
        // Initialize all hardware
        initializeRobot();


        // Show init status
        telemetry.addData("Status", "Initialized and Ready");
        telemetry.addData("Artifacts Loaded", artifactSorter.artifactCount);
        telemetry.addData("Backup Time", BACKUP_TIME + "s");
        telemetry.addData("Shooter RPM", SHOOTER_RPM);
        telemetry.addData("", "");
        telemetry.addData("Instructions", "Press Play to start");
        telemetry.update();

        waitForStart();
        drivetrain = new Drivetrain(hw, false);
        shooter = new Shooter2(hw, false);

        if (!opModeIsActive()) return;

        runtime.reset();
        currentState = AutoState.BACKUP;

        // Main autonomous loop
        while (opModeIsActive()) {
            double currentTime = getRuntime();

            // Always update these subsystems
            //sorter.updatePIDConstant();
            shooter.setPIDFCoefficients();

            // Run state machine
            switch (currentState) {
                case BACKUP:
                    runBackup(currentTime);
                    break;

                case SPIN_UP_SHOOTER:
                    runSpinUpShooter(currentTime);
                    break;

                case SHOOT_BALLS:
                    runShootBalls(currentTime);
                    break;

                case RETURN_HOME:
                    runReturnHome(currentTime);
                    break;

                case COMPLETE:
                    runComplete();
                    return; // Exit the loop
            }

            // Update telemetry
            updateTelemetry();
            sleep(10); // Small delay for stability
        }
    }

    private void initializeRobot() {
        // Initialize hardware
        hw = new RobotHardware();
        hw.init(hardwareMap);

        // Initialize subsystems
        drivetrain = new Drivetrain(hw, false);
        intake = new Intake(hw, false);
        sorter = new Sorter(hw, false);
        shooter = new Shooter2(hw, false);
        artifactSorter = new ArtifactSorter(hw, telemetry, sorter, false);
        stateMachine = new StateMachine(hw, sorter, shooter, artifactSorter, intake, false);

        // Pre-load artifacts (change colors if needed)
        artifactSorter.storedArtifacts[0] = "G"; // Slot 0: Green
        artifactSorter.storedArtifacts[1] = "P"; // Slot 1: Purple
        artifactSorter.storedArtifacts[2] = "P"; // Slot 2: Purple
        artifactSorter.artifactCount = 3;
        artifactSorter.nextSlotIndex = 0;
        artifactSorter.currentSlot = 0;
        artifactSorter.offSetApplied = false;

        // Ensure sorter starts at 0
        sorter.targetTicks = hw.sorter.getCurrentPosition();
    }

    // STATE: Backup
    private void runBackup(double currentTime) {
        if (currentState != AutoState.BACKUP) return;

        if (stateTimer.seconds() == 0) {
            stateTimer.reset();
            drivetrain.setMotorPowers(BACKUP_POWER, BACKUP_POWER, BACKUP_POWER, BACKUP_POWER);
        }

        if (stateTimer.seconds() >= BACKUP_TIME) {
            drivetrain.stop();
            transitionToState(AutoState.SPIN_UP_SHOOTER);
        }
    }

    // STATE: Spin up shooter
    private void runSpinUpShooter(double currentTime) {
        if (currentState != AutoState.SPIN_UP_SHOOTER) return;

        if (stateTimer.seconds() == 0) {
            stateTimer.reset();
            shooter.setTargetVelRPM(SHOOTER_RPM);
        }

        // Check if shooter is at speed
        if (shooter.atTargetVel()) {
            transitionToState(AutoState.SHOOT_BALLS);
        }

        // Timeout after 5 seconds
        if (stateTimer.seconds() > 5.0) {
            telemetry.addData("Warning", "Shooter didn't reach speed, continuing anyway");
            transitionToState(AutoState.SHOOT_BALLS);
        }
    }

    // STATE: Shoot all balls using state machine
    private int ballsShot = 0;
    private boolean shootingStarted = false;

    private void runShootBalls(double currentTime) {
        if (currentState != AutoState.SHOOT_BALLS) return;

        if (!shootingStarted) {
            stateTimer.reset();
            shootingStarted = true;
            ballsShot = 0;
        }

        // Use the state machine's shooting logic
        stateMachine.updateShooting(false, true, currentTime); // true for close shot

        // Check if we're done shooting
        if (artifactSorter.artifactCount == 0 && stateMachine.shootState == StateMachine.ShootState.IDLE) {
            transitionToState(AutoState.RETURN_HOME);
            shootingStarted = false;
        }

        // Timeout after 15 seconds
        if (stateTimer.seconds() > 15.0) {
            telemetry.addData("Warning", "Shooting timeout");
            transitionToState(AutoState.RETURN_HOME);
            shootingStarted = false;
        }
    }

    // STATE: Return sorter to home
    private boolean returningHome = false;

    private void runReturnHome(double currentTime) {
        if (currentState != AutoState.RETURN_HOME) return;

        if (!returningHome) {
            stateTimer.reset();
            returningHome = true;

            // Return to home if offset was applied
            if (artifactSorter.offSetApplied) {
                sorter.moveDegrees(75);
            } else {
                // Already home
                transitionToState(AutoState.COMPLETE);
                return;
            }
        }

        // Wait for sorter to reach home
        if (sorter.atTarget()) {
            transitionToState(AutoState.COMPLETE);
        }

        // Timeout after 3 seconds
        if (stateTimer.seconds() > 3.0) {
            transitionToState(AutoState.COMPLETE);
        }
    }

    // STATE: Complete and cleanup
    private void runComplete() {
        shooter.setTargetVelRPM(0);
        drivetrain.stop();
        sorter.resetPID();
        artifactSorter.resetSlot();
        drivetrain.setMotorPowers(0.6, 0.6, 0.6, 0.6);
        sleep(1000);
        drivetrain.stop();
        telemetry.addData("Status", "COMPLETE!");
        telemetry.addData("Total Runtime", "%.1f seconds", runtime.seconds());
        telemetry.addData("", "");
        telemetry.addData("Good job!", "Auto finished successfully");
        telemetry.update();

        sleep(1000);
    }

    // Helper method to transition between states
    private void transitionToState(AutoState newState) {
        currentState = newState;
        stateTimer.reset();
    }

    // Telemetry update
    private void updateTelemetry() {
        telemetry.addData("State", currentState);
        telemetry.addData("Runtime", "%.1f s", runtime.seconds());
        telemetry.addData("State Time", "%.1f s", stateTimer.seconds());
        telemetry.addData("", "");

        // Drivetrain info
        telemetry.addData("Backup Time", "%.1f / %.1f s", stateTimer.seconds(), BACKUP_TIME);
        telemetry.addData("", "");

        // Shooter info
        telemetry.addData("Shooter RPM", "%.0f / %.0f", shooter.currentVelocity(), SHOOTER_RPM);
        telemetry.addData("Shooter Ready", shooter.atTargetVel() ? "YES" : "NO");
        telemetry.addData("", "");

        // Sorter info
        telemetry.addData("Sorter Pos", hw.sorter.getCurrentPosition());
        telemetry.addData("Sorter Target", sorter.targetTicks);
        telemetry.addData("Sorter At Target", sorter.atTarget() ? "YES" : "NO");
        telemetry.addData("", "");

        // Artifact info
        telemetry.addData("Artifacts Left", artifactSorter.artifactCount);
        telemetry.addData("Current Slot", artifactSorter.currentSlot);
        telemetry.addData("Shoot State", stateMachine.shootState);

        telemetry.update();
    }
}