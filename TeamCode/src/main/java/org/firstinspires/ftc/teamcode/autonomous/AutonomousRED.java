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

    private RobotHardware hw;
    private Drivetrain drivetrain;
    private Sorter sorter;
    private Shooter2 shooter;
    private Intake intake;
    private ArtifactSorter artifactSorter;
    private StateMachine stateMachine;
    private Follower follower;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stateTimer = new ElapsedTime();

    private final double SHOOTER_RPM = 2700; // RED autonomous RPM

    // POSITIONS
    private final Pose startPose    = new Pose(0, 0, Math.toRadians(45));
    private final Pose shootingPose = new Pose(-30, -30, Math.toRadians(45));
    private final Pose pos1         = new Pose(-35, -40, Math.toRadians(0));
    private final Pose pos1Forward  = new Pose(-2.3, -36.8, Math.toRadians(0));
    private final Pose pos2         = new Pose(-35, -61, Math.toRadians(0));
    private final Pose pos2Forward  = new Pose(-2.3, -61, Math.toRadians(0));

    private boolean isMoving    = false;
    private PathChain currentPath = null;
    private int currentCycle = 0;

    // Sorter pause
    private int lastArtifactCount  = 0;
    private boolean waitingForSorter = false;

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

    private AutoState currentState = AutoState.INIT;

    @Override
    public void runOpMode() {
        initializeRobot();

        telemetry.addData("Status", "opmode running");
        telemetry.addData("Artifacts", artifactSorter.artifactCount);
        telemetry.addData("Shooter RPM", SHOOTER_RPM);
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        runtime.reset();
        currentState = AutoState.GO_TO_SHOOTING_POS;
        currentCycle = 0;
        follower.setMaxPower(0.8);

        while (opModeIsActive()) {
            double currentTime = getRuntime();
            follower.update();

            sorter.update();
            shooter.setPIDFCoefficients();

            if (currentState == AutoState.COLLECT_BALLS) {
                stateMachine.updateIntake(false, false, currentTime, false, false);
            }

            switch (currentState) {
                case GO_TO_SHOOTING_POS: runGoToShootingPos(currentTime);  break;
                case SPIN_UP_SHOOTER:    runSpinUpShooter(currentTime);    break;
                case SHOOT_BALLS:        runShootBalls(currentTime);       break;
                case GO_TO_COLLECTION:   runGoToCollection(currentTime);   break;
                case COLLECT_BALLS:      runCollectBalls(currentTime);     break;
                case RETURN_TO_SHOOT:    runReturnToShoot(currentTime);    break;
                case RETURN_HOME:        runReturnHome(currentTime);       break;
                case COMPLETE:           runComplete();                     return;
            }

            updateTelemetry();
            sleep(10);
        }
    }

    private void initializeRobot() {
        hw = new RobotHardware();
        hw.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        drivetrain    = new Drivetrain(hw, false);
        intake        = new Intake(hw, false);
        sorter        = new Sorter(hw, false);
        shooter       = new Shooter2(hw, false);
        artifactSorter = new ArtifactSorter(hw, telemetry, sorter, false);

        // Initialize StateMachine with RED autonomous selection
        stateMachine  = new StateMachine(hw, sorter, shooter, artifactSorter, intake, false, StateMachine.AutoColor.RED);

        // Preset artifact configuration
        artifactSorter.storedArtifacts[0] = "G";
        artifactSorter.storedArtifacts[1] = "P";
        artifactSorter.storedArtifacts[2] = "P";
        artifactSorter.artifactCount  = 3;
        artifactSorter.nextSlotIndex  = 0;
        artifactSorter.currentSlot    = 0;
        artifactSorter.offSetApplied  = false;

        sorter.targetTicks = hw.sorter.getCurrentPosition();
    }

    // =================== PEDRO PATHING ===================

    private void goToPose(Pose targetPose, String interpolationType) {
        Pose currentPose = follower.getPose();
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

    private boolean hasReachedTarget() {
        if (!isMoving) return true;
        if (!follower.isBusy()) {
            isMoving = false;
            currentPath = null;
            return true;
        }
        return false;
    }

    // =================== AUTO STATES ===================

    private void runGoToShootingPos(double currentTime) {
        if (!isMoving) {
            stateTimer.reset();
            goToPose(shootingPose, "constant");
        }
        if (hasReachedTarget() || stateTimer.seconds() > 5.0)
            transitionToState(AutoState.SPIN_UP_SHOOTER);
    }

    private boolean shooterStarted = false;
    private void runSpinUpShooter(double currentTime) {
        if (!shooterStarted) {
            stateTimer.reset();
            shooter.setTargetVelRPM(SHOOTER_RPM);
            shooterStarted = true;
        }
        if (shooter.atTargetVel() || stateTimer.seconds() > 5.0) {
            shooterStarted = false;
            transitionToState(AutoState.SHOOT_BALLS);
        }
    }

    private boolean shootingStarted = false;
    private void runShootBalls(double currentTime) {
        if (!shootingStarted) shootingStarted = true;
        stateMachine.updateShooting(false, true, currentTime);

        if (artifactSorter.artifactCount == 0
                && stateMachine.getShootState() == StateMachine.ShootState.IDLE) {
            shootingStarted = false;
            if (currentCycle < 2) {
                currentCycle++;
                transitionToState(AutoState.GO_TO_COLLECTION);
            } else {
                transitionToState(AutoState.RETURN_HOME);
            }
        }

        if (stateTimer.seconds() > 15.0) {
            shootingStarted = false;
            transitionToState(AutoState.RETURN_HOME);
        }
    }

    private void runGoToCollection(double currentTime) {
        if (!isMoving) {
            stateTimer.reset();
            if (currentCycle == 1) goToPose(pos1, "linear");
            else if (currentCycle == 2) goToPose(pos2, "linear");
        }
        if (hasReachedTarget() || stateTimer.seconds() > 5.0)
            transitionToState(AutoState.COLLECT_BALLS);
    }

    private boolean collectingStarted = false;
    private void runCollectBalls(double currentTime) {
        if (!collectingStarted) {
            stateTimer.reset();
            collectingStarted = true;
            lastArtifactCount = artifactSorter.artifactCount;
            waitingForSorter  = false;
            intake.intakeOn   = true;
            follower.setMaxPower(0.3);

            if (currentCycle == 1)      goToPose(pos1Forward, "constant");
            else if (currentCycle == 2) goToPose(pos2Forward, "constant");
        }

        // Pause follower if sorter is moving
        if (artifactSorter.artifactCount > lastArtifactCount) {
            lastArtifactCount = artifactSorter.artifactCount;
            waitingForSorter  = true;
            follower.setMaxPower(0);
        }

        if (waitingForSorter && sorter.atTarget()) {
            waitingForSorter = false;
            follower.setMaxPower(0.2);
        }

        boolean collectionDone = artifactSorter.artifactCount >= 3
                && (stateMachine.getIntakeState() == StateMachine.IntakeState.IDLE
                || stateMachine.getIntakeState() == StateMachine.IntakeState.FULL);

        if (collectionDone || stateTimer.seconds() > 5.0) {
            intake.intakeOn      = false;
            collectingStarted    = false;
            waitingForSorter     = false;
            follower.setMaxPower(0.8);
            isMoving = false;
            transitionToState(AutoState.RETURN_TO_SHOOT);
        }

        stateMachine.updateIntake(false, false, currentTime, false, false);
    }

    private void runReturnToShoot(double currentTime) {
        if (!isMoving) {
            stateTimer.reset();
            if (follower.isBusy()) {
                if (stateTimer.seconds() > 0.15) follower.setMaxPower(0.2);
                return;
            }
            follower.setMaxPower(0.8);
            goToPose(shootingPose, "linear");
        }
        if (hasReachedTarget() || stateTimer.seconds() > 5.0)
            transitionToState(AutoState.SPIN_UP_SHOOTER);
    }

    private boolean returningHome = false;
    private void runReturnHome(double currentTime) {
        if (!returningHome) {
            stateTimer.reset();
            returningHome = true;
            if (artifactSorter.offSetApplied) sorter.moveDegrees(60);
            else { returningHome = false; transitionToState(AutoState.COMPLETE); return; }
        }
        if (sorter.atTarget() || stateTimer.seconds() > 3.0) {
            returningHome = false;
            transitionToState(AutoState.COMPLETE);
        }
    }

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

    private void transitionToState(AutoState newState) {
        currentState = newState;
        stateTimer.reset();
        isMoving = false;
    }

    private void updateTelemetry() {
        telemetry.addData("State", currentState);
        telemetry.addData("Cycle", currentCycle);
        telemetry.addData("Runtime", "%.1f s", runtime.seconds());
        telemetry.addData("WaitingForSorter", waitingForSorter);
        telemetry.addData("X", "%.1f", follower.getPose().getX());
        telemetry.addData("Y", "%.1f", follower.getPose().getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Shooter RPM", "%.0f / %.0f", shooter.currentVelocity(), SHOOTER_RPM);
        telemetry.addData("Shooter Ready", shooter.atTargetVel() ? "YES" : "NO");
        telemetry.addData("Sorter Pos", hw.sorter.getCurrentPosition());
        telemetry.addData("Sorter Target", sorter.targetTicks);
        telemetry.addData("Sorter At Target", sorter.atTarget() ? "YES" : "NO");
        telemetry.addData("Artifacts Left", artifactSorter.artifactCount);
        telemetry.addData("Current Slot", artifactSorter.currentSlot);
        telemetry.addData("Shoot State", stateMachine.getShootState());
        telemetry.addData("Intake State", stateMachine.getIntakeState());
        telemetry.addData("Intake On", intake.intakeOn);
        telemetry.update();
    }
}