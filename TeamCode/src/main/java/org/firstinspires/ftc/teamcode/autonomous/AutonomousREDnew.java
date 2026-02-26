package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArtifactSystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter2;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;

@Autonomous(name = "Autonomous RED")
public class AutonomousREDnew extends LinearOpMode {

    // Hardware and subsystems
    RobotHardware     hw;
    Drivetrain        drivetrain;
    Sorter            sorter;
    Shooter2          shooter;
    Intake            intake;
    ArtifactSystem    artifactSystem;
    private Follower  follower;

    ElapsedTime runtime    = new ElapsedTime();
    ElapsedTime stateTimer = new ElapsedTime();

    // POSITIONS
    private final Pose startPose    = new Pose(0, 0, Math.toRadians(45));
    private final Pose shootingPose = new Pose(-30, -30, Math.toRadians(45));
    private final Pose pos1         = new Pose(-30, -40, Math.toRadians(0));
    private final Pose pos1Forward  = new Pose(-2.3, -36.8, Math.toRadians(0));
    private final Pose pos2         = new Pose(-30, -61, Math.toRadians(0));
    private final Pose pos2Forward  = new Pose(-2.3, -61, Math.toRadians(0));

    // ArtifactSystem control flags
    // They are automatically cleared at the bottom of each loop iteration.
    private boolean dpadUp        = false;
    private boolean dpadDown      = false;
    private boolean dpadLeft      = false;
    private boolean dpadRight     = false;
    private boolean leftTrigger   = false;
    private boolean rightTrigger  = false;
    private boolean leftBumper    = false;
    private boolean rightBumper   = false;
    private boolean intakeToggle  = false;
    private boolean servoTransfer = false;
    private boolean intakeReverse = false;
    private boolean buttonB       = false; // auto fire all

    // Movement tracking
    private boolean   isMoving    = false;
    private PathChain currentPath = null;

    // Cycle tracking: 0 = initial shoot, 1 = pos1 cycle, 2 = pos2 cycle
    private int currentCycle = 0;

    // State machine flags
    private boolean shootingPosStarted = false;
    private boolean collectingStarted  = false;

    // State machine
    enum AutoState {
        INIT,
        GO_TO_SHOOTING_POS,
        GO_TO_COLLECTION,
        COLLECT_BALLS,
        RETURN_TO_SHOOT,
        RETURN_HOME,
        COMPLETE
    }

    AutoState currentState = AutoState.INIT;

    @Override
    public void runOpMode() {
        initializeRobot();

        telemetry.addData("Status", "Ready");
        telemetry.addData("Artifacts preloaded", artifactSystem.artifactCount);
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) return;

        runtime.reset();
        currentState = AutoState.GO_TO_SHOOTING_POS;
        currentCycle = 0;
        follower.setMaxPower(0.8);

        while (opModeIsActive()) {
            double currentTime = getRuntime();

            // Always update subsystems every loop
            follower.update();
            sorter.update();
            shooter.setPIDFCoefficients();

            // Run state machine
            switch (currentState) {
                case GO_TO_SHOOTING_POS:
                    runGoToShootingPos();
                    break;
                case GO_TO_COLLECTION:
                    runGoToCollection();
                    break;
                case COLLECT_BALLS:
                    runCollectBalls();
                    break;
                case RETURN_TO_SHOOT:
                    runReturnToShoot();
                    break;
                case RETURN_HOME:
                    runReturnHome();
                    break;
                case COMPLETE:
                    runComplete();
                    return;
            }

            // Feed raw flags directly into ArtifactSystem
            // ArtifactSystem handles its own edge detection...
            artifactSystem.update(
                    dpadUp,
                    dpadDown,
                    dpadLeft,
                    dpadRight,
                    leftTrigger,        // continuous
                    rightTrigger,       // continuous
                    leftBumper,
                    rightBumper,
                    intakeToggle,
                    servoTransfer,
                    intakeReverse,
                    buttonB,
                    currentTime
            );

            // Clear all flags for the next loop
            dpadUp        = false;
            dpadDown      = false;
            dpadLeft      = false;
            dpadRight     = false;
            leftBumper    = false;
            rightBumper   = false;
            intakeToggle  = false;
            servoTransfer = false;
            intakeReverse = false;
            buttonB       = false;

            updateTelemetry();
            sleep(10);
        }
    }

    private void initializeRobot() {
        hw = new RobotHardware();
        hw.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        drivetrain = new Drivetrain(hw, follower, true);
        intake         = new Intake(hw, false);
        sorter         = new Sorter(hw, false);
        shooter        = new Shooter2(hw, false);
        artifactSystem = new ArtifactSystem(hw, telemetry, sorter, shooter, intake, false);

        // Preload
        artifactSystem.storedArtifacts[0] = "G";
        artifactSystem.storedArtifacts[1] = "P";
        artifactSystem.storedArtifacts[2] = "P";
        artifactSystem.artifactCount  = 3;
        artifactSystem.nextSlotIndex  = 0;
        artifactSystem.currentSlot    = 0;
        artifactSystem.offSetApplied  = false;
    }

    // =================== PEDRO PATHING ===================

    private void goToPosition(double x, double y, double heading, String interpolationType) {
        Pose currentPose = follower.getPose();
        Pose targetPose  = new Pose(x, y, Math.toRadians(heading));

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
        isMoving    = true;
    }

    private void goToPose(Pose targetPose, String interpolationType) {
        goToPosition(
                targetPose.getX(),
                targetPose.getY(),
                Math.toDegrees(targetPose.getHeading()),
                interpolationType
        );
    }

    private boolean hasReachedTarget() {
        if (!isMoving) return true;

        // Added a 100ms buffer to allow PedroPathing to register the new path as "busy"
        if (stateTimer.milliseconds() > 100 && !follower.isBusy()) {
            isMoving    = false;
            currentPath = null;
            return true;
        }
        return false;
    }

    // ================== STATE FUNCTIONS ====================

    // STATE: Drive to shooting position, then trigger auto fire
    private void runGoToShootingPos() {
        if (currentState != AutoState.GO_TO_SHOOTING_POS) return;

        // Ensure we only initiate the drive once per cycle
        if (!shootingPosStarted) {
            stateTimer.reset();
            goToPose(shootingPose, "constant");
            dpadLeft = true; // enter SHOOT state
            shootingPosStarted = true;
        }

        // Wait until reached target and then fire
        if (hasReachedTarget()) {
            buttonB = true; // auto fire all pulse

            // Wait until ArtifactSystem finishes shooting
            if (!artifactSystem.isActivelyShooting()) {
                shootingPosStarted = false; // Reset the flag for the next cycle

                if (currentCycle == 0) {
                    currentCycle = 1;
                    transitionToState(AutoState.GO_TO_COLLECTION);
                } else if (currentCycle == 1) {
                    currentCycle = 2;
                    transitionToState(AutoState.GO_TO_COLLECTION);
                } else {
                    transitionToState(AutoState.RETURN_HOME);
                }
            }
        }

        // Safety timeout
        if (stateTimer.seconds() > 20.0) {
            isMoving = false;
            shootingPosStarted = false;
            transitionToState(currentCycle < 2 ? AutoState.GO_TO_COLLECTION : AutoState.RETURN_HOME);
        }
    }

    // STATE: Drive to collection position and turn intake on
    private void runGoToCollection() {
        if (currentState != AutoState.GO_TO_COLLECTION) return;

        if (!isMoving) {
            stateTimer.reset();

            if (currentCycle == 1) {
                goToPose(pos1, "linear");
            } else if (currentCycle == 2) {
                goToPose(pos2, "linear");
            }

            dpadRight    = true; // exit SHOOT state
            intakeToggle = true; // turn intake on
        }

        if (hasReachedTarget()) {
            transitionToState(AutoState.COLLECT_BALLS);
        }

        if (stateTimer.seconds() > 5.0) {
            isMoving = false;
            transitionToState(AutoState.COLLECT_BALLS);
        }
    }

    // STATE: Inch forward slowly while collecting
    private void runCollectBalls() {
        if (currentState != AutoState.COLLECT_BALLS) return;

        if (!collectingStarted) {
            stateTimer.reset();
            collectingStarted = true;
            follower.setMaxPower(0.4 * (12.0 / hw.getBatteryVoltage()));

            if (currentCycle == 1) {
                goToPose(pos1Forward, "linear");
            } else if (currentCycle == 2) {
                goToPose(pos2Forward, "linear");
            }
        }

        // Leave once we have a full load, reached the forward pose, or timed out
        if (artifactSystem.artifactCount >= 3 || hasReachedTarget() || stateTimer.seconds() > 5.0) {
            intakeToggle      = true; // turn intake off pulse
            collectingStarted = false;
            transitionToState(AutoState.RETURN_TO_SHOOT);
        }
    }

    // STATE: Return to shooting position and spin up
    private void runReturnToShoot() {
        if (currentState != AutoState.RETURN_TO_SHOOT) return;

        if (!isMoving) {
            stateTimer.reset();
            follower.setMaxPower(0.8 * (12.0 / hw.getBatteryVoltage()));
            goToPose(shootingPose, "linear");
            dpadLeft = true; // re-enter SHOOT state
        }

        if (hasReachedTarget()) {
            transitionToState(AutoState.GO_TO_SHOOTING_POS);
        }

        if (stateTimer.seconds() > 5.0) {
            isMoving = false;
            transitionToState(AutoState.GO_TO_SHOOTING_POS);
        }
    }

    // STATE: Park / return home
    private void runReturnHome() {
        if (currentState != AutoState.RETURN_HOME) return;

        if (!isMoving) {
            stateTimer.reset();
            goToPose(startPose, "linear");
        }

        if (hasReachedTarget()) {
            transitionToState(AutoState.COMPLETE);
        }

        if (stateTimer.seconds() > 5.0) {
            isMoving = false;
            transitionToState(AutoState.COMPLETE);
        }
    }

    // STATE: Cleanup and stop
    private void runComplete() {
        shooter.setTargetVelRPM(0);
        drivetrain.stop();
        sorter.resetPID();
        artifactSystem.resetSlot();

        telemetry.addData("Status", "Complete");
        telemetry.addData("Total Runtime", "%.1f seconds", runtime.seconds());
        telemetry.addData("Cycles Completed", currentCycle);
        telemetry.update();

        sleep(1000);
    }

    private void transitionToState(AutoState newState) {
        currentState = newState;
        stateTimer.reset();
    }

    private void updateTelemetry() {
        telemetry.addData("State",       currentState);
        telemetry.addData("Cycle",       currentCycle + " (0=Init, 1=Pos1, 2=Pos2)");
        telemetry.addData("Runtime",     "%.1f s", runtime.seconds());
        telemetry.addData("State Time",  "%.1f s", stateTimer.seconds());
        telemetry.addData("", "");
        telemetry.addData("X",           "%.1f in", follower.getPose().getX());
        telemetry.addData("Y",           "%.1f in", follower.getPose().getY());
        telemetry.addData("Heading",     "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path Busy",   follower.isBusy());
        telemetry.addData("Is Moving",   isMoving);
        telemetry.addData("", "");
        telemetry.addData("Artifacts",   artifactSystem.artifactCount);
        telemetry.addData("Robot State", artifactSystem.robotState);
        telemetry.addData("Shoot State", artifactSystem.shootSubState);
        telemetry.addData("Shooter RPM", shooter.currentVelocity());
        telemetry.update();
    }
}