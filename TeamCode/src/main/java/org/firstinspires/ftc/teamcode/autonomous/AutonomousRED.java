package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArtifactSorter;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter2;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;
import org.firstinspires.ftc.teamcode.subsystems.StateMachine;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "Autonomous RED")
public class AutonomousRED extends LinearOpMode {

    // Hardware and subsystems
    RobotHardware hw;
    Drivetrain drivetrain;
    Sorter sorter;
    Shooter2 shooter;
    Intake intake;
    ArtifactSorter artifactSorter;
    StateMachine stateMachine;
    private Follower follower;

    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime stateTimer = new ElapsedTime();

    // Tunable parameters
    double SHOOTER_RPM = 1300;

    // POSITIONS
    private final Pose startPose = new Pose(0, 0, Math.toRadians(45));
    private final Pose shootingPose = new Pose(-30, -30, Math.toRadians(45));
    private final Pose pos1 = new Pose(-19.3, -36.8, Math.toRadians(0));
    private final Pose pos1Forward = new Pose(-2.3, -36.8, Math.toRadians(0));
    private final Pose pos2 = new Pose(-19.3, -61, Math.toRadians(0));
    private final Pose pos2Forward = new Pose(-2.3, -61, Math.toRadians(0));

    // Movement tracking
    private boolean isMoving = false;
    private PathChain currentPath = null;

    // Cycle tracking
    private int currentCycle = 0; // 0 = initial shoot, 1 = pos1 cycle, 2 = pos2 cycle

    // State machine for autonomous
    enum AutoState {
        INIT,
        GO_TO_SHOOTING_POS,
        SPIN_UP_SHOOTER,
        SHOOT_BALLS,
        GO_TO_COLLECTION,
        COLLECT_BALLS,
        RETURN_TO_SHOOT,
        RETURN_HOME,
        COMPLETE
    }

    AutoState currentState = AutoState.INIT;

    @Override
    public void runOpMode() {
        // Initialize all hardware
        initializeRobot();

        // Show init status
        telemetry.addData("Status", "opmode running");
        telemetry.addData("Artifacts", artifactSorter.artifactCount);
        telemetry.addData("Shooter rpm", SHOOTER_RPM);
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) return;

        runtime.reset();
        currentState = AutoState.GO_TO_SHOOTING_POS; // First state
        currentCycle = 0;
        follower.setMaxPower(0.8);

        // Main autonomous loop
        while (opModeIsActive()) {
            double currentTime = getRuntime();
            follower.update();

            // Always update these subsystems
            sorter.update();
            shooter.setPIDFCoefficients();

            // Update intake state machine during collection
            if (currentState == AutoState.COLLECT_BALLS) {
                stateMachine.updateIntake(false, false, currentTime, false, false);
            }

            // Run state machine
            switch (currentState) {
                case GO_TO_SHOOTING_POS:
                    runGoToShootingPos(currentTime);
                    break;

                case SPIN_UP_SHOOTER:
                    runSpinUpShooter(currentTime);
                    break;

                case SHOOT_BALLS:
                    runShootBalls(currentTime);
                    break;

                case GO_TO_COLLECTION:
                    runGoToCollection(currentTime);
                    break;

                case COLLECT_BALLS:
                    runCollectBalls(currentTime);
                    break;

                case RETURN_TO_SHOOT:
                    runReturnToShoot(currentTime);
                    break;

                case RETURN_HOME:
                    runReturnHome(currentTime);
                    break;

                case COMPLETE:
                    runComplete();
                    return; // Exit the loop
            }

            updateTelemetry(); // Update telemetry
            sleep(10); // Small delay for stability
        }
    }

    private void initializeRobot() {
        // Initialize hardware
        hw = new RobotHardware();
        hw.init(hardwareMap);

        // Initialize PedroPathing
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize subsystems
        drivetrain = new Drivetrain(hw, false);
        intake = new Intake(hw, false);
        sorter = new Sorter(hw, false);
        shooter = new Shooter2(hw, false);
        artifactSorter = new ArtifactSorter(hw, telemetry, sorter, false);
        stateMachine = new StateMachine(hw, sorter, shooter, artifactSorter, intake, false);

        // Preload artifacts
        artifactSorter.storedArtifacts[0] = "G";
        artifactSorter.storedArtifacts[1] = "P";
        artifactSorter.storedArtifacts[2] = "P";
        artifactSorter.artifactCount = 3;
        artifactSorter.nextSlotIndex = 0;
        artifactSorter.currentSlot = 0;
        artifactSorter.offSetApplied = false;

        // reset sorter to 0
        sorter.targetTicks = hw.sorter.getCurrentPosition();
    }

    // =================== PEDRO PATHING FUNCTIONS=================

    /**
     * Start moving to a target position using a straight line
     * @param x Target X coordinate
     * @param y Target Y coordinate
     * @param heading Target heading in degrees
     */
    private void goToPosition(double x, double y, double heading) {
        goToPosition(x, y, heading, "linear");
    }

    /**
     * Start moving to a target position with custom heading interpolation
     * @param x Target X coordinate
     * @param y Target Y coordinate
     * @param heading Target heading in degrees
     * @param interpolationType "constant", "linear", or "tangent"
     */
    private void goToPosition(double x, double y, double heading, String interpolationType) {
        Pose currentPose = follower.getPose();
        Pose targetPose = new Pose(x, y, Math.toRadians(heading));

        PathChain path;

        switch (interpolationType.toLowerCase()) {
            case "constant":
                path = follower.pathBuilder()
                        .addPath(new BezierLine(currentPose, targetPose))
                        .setConstantHeadingInterpolation(currentPose.getHeading())
                        .build();
                break;
            case "tangent":
                path = follower.pathBuilder()
                        .addPath(new BezierLine(currentPose, targetPose))
                        .setTangentHeadingInterpolation()
                        .build();
                break;
            case "linear":
            default:
                path = follower.pathBuilder()
                        .addPath(new BezierLine(currentPose, targetPose))
                        .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
                        .build();
                break;
        }

        follower.followPath(path);
        currentPath = path;
        isMoving = true;
    }

    /**
     * Go to a specific Pose (easier to use with predefined positions)
     */
    private void goToPose(Pose targetPose, String interpolationType) {
        goToPosition(targetPose.getX(), targetPose.getY(), Math.toDegrees(targetPose.getHeading()), interpolationType);
    }

    /**
     * Check if robot has finished moving to target position
     * @return true if movement is complete
     */
    private boolean hasReachedTarget() {
        if (!isMoving) return true;

        if (!follower.isBusy()) {
            isMoving = false;
            currentPath = null;
            return true;
        }
        return false;
    }

    // ================== AUTO STATE FUNCTIONS ====================

    // STATE: Go to shooting position
    private void runGoToShootingPos(double currentTime) {
        if (currentState != AutoState.GO_TO_SHOOTING_POS) return;

        if (!isMoving) {
            stateTimer.reset();
            goToPose(shootingPose, "constant");
            telemetry.addData("Status", "shoot pos");
            telemetry.update();
        }

        if (hasReachedTarget()) {
            transitionToState(AutoState.SPIN_UP_SHOOTER);
        }

        // Timeout
        if (stateTimer.seconds() > 5.0) {
            isMoving = false;
            transitionToState(AutoState.SPIN_UP_SHOOTER);
        }
    }

    // STATE: Spin up shooter
    private boolean shooterStarted = false;

    private void runSpinUpShooter(double currentTime) {
        if (currentState != AutoState.SPIN_UP_SHOOTER) return;

        if (!shooterStarted) {
            stateTimer.reset();
            shooter.setTargetVelRPM(SHOOTER_RPM);
            shooterStarted = true;
        }

        // Check if shooter is at speed
        if (shooter.atTargetVel()) { // BUG WITH ROBOT 2
            shooterStarted = false;
            transitionToState(AutoState.SHOOT_BALLS);
        }

        // Timeout after 5 seconds
        if (stateTimer.seconds() > 5.0) {
            shooterStarted = false;
            telemetry.addData("Warning", "Shooter didnt reach speed so continuing anyway");
            transitionToState(AutoState.SHOOT_BALLS);
        }
    }

    // STATE: Shoot all balls
    private boolean shootingStarted = false;

    private void runShootBalls(double currentTime) {
        if (currentState != AutoState.SHOOT_BALLS) return;

        if (!shootingStarted) {
            stateTimer.reset();
            shootingStarted = true;
        }

        //state machine shooting logic
        stateMachine.updateShooting(false, true, currentTime);

        // Check if done shooting
        if (artifactSorter.artifactCount == 0 && stateMachine.shootState == StateMachine.ShootState.IDLE) {
            shootingStarted = false;

            // check if if pos 1 or pos 2 should be collected
            if (currentCycle == 0) {
                // collect pos1
                currentCycle = 1;
                transitionToState(AutoState.GO_TO_COLLECTION);
            } else if (currentCycle == 1) {
                // collect po2
                currentCycle = 2;
                transitionToState(AutoState.GO_TO_COLLECTION);
            } else {
                // return home
                transitionToState(AutoState.RETURN_HOME);
            }
        }

        // Timeout after 15 seconds
        if (stateTimer.seconds() > 15.0) {
            telemetry.addData("Warning", "Shooting timeout");
            shootingStarted = false;
            transitionToState(AutoState.RETURN_HOME);
        }
    }

    // STATE: Go to collection position
    private void runGoToCollection(double currentTime) {
        if (currentState != AutoState.GO_TO_COLLECTION) return;

        if (!isMoving) {
            stateTimer.reset();

            // Go to the appropriate collection position based on cycle
            if (currentCycle == 1) {
                goToPose(pos1, "linear");
                telemetry.addData("Status", "pos1");
            } else if (currentCycle == 2) {
                goToPose(pos2, "linear");
                telemetry.addData("Status", "pos2");
            }

            telemetry.update();
        }

        if (hasReachedTarget()) {
            transitionToState(AutoState.COLLECT_BALLS);
        }

        // Timeout
        if (stateTimer.seconds() > 5.0) {
            isMoving = false;
            transitionToState(AutoState.COLLECT_BALLS);
        }
    }

    // STATE: Collect balls (move forward and run intake)
    private boolean collectingStarted = false;

    private void runCollectBalls(double currentTime) {
        if (currentState != AutoState.COLLECT_BALLS) return;

        if (!collectingStarted) {
            stateTimer.reset();
            collectingStarted = true;

            // Start intake using your intake system
            intake.intakeOn = true;

            // Move forward to pos forward
            if (currentCycle == 1) {
                follower.setMaxPower(0.4);
                goToPose(pos1Forward, "constant");
                telemetry.addData("Status", "pos1 collecting");
            } else if (currentCycle == 2) {
                follower.setMaxPower(0.4);
                goToPose(pos2Forward, "constant");
                telemetry.addData("Status", "pos2 collecting");
            }

            telemetry.update();
        }

        // Check if collected
        if (artifactSorter.artifactCount >= 3) {
            intake.intakeOn = false;
            collectingStarted = false;
            transitionToState(AutoState.RETURN_TO_SHOOT);
        }

        // Timeout after 5 seconds (I hope this has time lel)
        if (stateTimer.seconds() > 5.0) {
            intake.intakeOn = false;
            collectingStarted = false;
            transitionToState(AutoState.RETURN_TO_SHOOT);
        }
    }

    // STATE: Return to shooting position
    private void runReturnToShoot(double currentTime) {
        if (currentState != AutoState.RETURN_TO_SHOOT) return;

        if (!isMoving) {
            stateTimer.reset();
            follower.setMaxPower(0.8);
            goToPose(shootingPose, "constant"); // Go back to shooting position
            telemetry.addData("Status", "return to shootstate");
            telemetry.update();
        }

        if (hasReachedTarget()) {
            transitionToState(AutoState.SPIN_UP_SHOOTER);
        }

        // Timeout
        if (stateTimer.seconds() > 5.0) {
            isMoving = false;
            transitionToState(AutoState.SPIN_UP_SHOOTER);
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
                sorter.moveDegrees(60);
            } else {
                // Already home
                returningHome = false;
                transitionToState(AutoState.COMPLETE);
                return;
            }
        }

        // Wait for sorter to reach home
        if (sorter.atTarget()) {
            returningHome = false;
            transitionToState(AutoState.COMPLETE);
        }

        // Timeout after 3 seconds
        if (stateTimer.seconds() > 3.0) {
            returningHome = false;
            transitionToState(AutoState.COMPLETE);
        }
    }

    // STATE: Complete and cleanup
    private void runComplete() {
        shooter.setTargetVelRPM(0);
        drivetrain.stop();
        sorter.resetPID();
        artifactSorter.resetSlot();

        telemetry.addData("Status", "finish");
        telemetry.addData("Total Runtime", "%.1f seconds", runtime.seconds());
        telemetry.addData("Cycles Completed", currentCycle);
        telemetry.update();

        sleep(1000);
    }

    // method to transition between states
    private void transitionToState(AutoState newState) {
        currentState = newState;
        stateTimer.reset();
    }

    // Telemetry update //damnn chatgpt cooking with debug things
    private void updateTelemetry() {
        telemetry.addData("State", currentState);
        telemetry.addData("Cycle", currentCycle + " (0=Init, 1=Pos1, 2=Pos2)");
        telemetry.addData("Runtime", "%.1f s", runtime.seconds());
        telemetry.addData("State Time", "%.1f s", stateTimer.seconds());
        telemetry.addData("", "");

        // Robot position from PedroPathing
        telemetry.addData("X Position", "%.1f in", follower.getPose().getX());
        telemetry.addData("Y Position", "%.1f in", follower.getPose().getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path Busy", follower.isBusy());
        telemetry.addData("Is Moving", isMoving);
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
        telemetry.addData("Intake State", stateMachine.intakeState);
        telemetry.addData("Intake On", intake.intakeOn);

        telemetry.update();
    }
}