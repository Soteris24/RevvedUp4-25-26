package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

public abstract class BaseAutonomous1 extends LinearOpMode {
    protected abstract Pose getStartPose();
    protected abstract Pose getShootingPose();
    protected abstract Pose getPos1();
    protected abstract Pose getPos1Forward();
    protected abstract Pose getPos2();
    protected abstract Pose getPos2Forward();
    RobotHardware hw;
    Drivetrain drivetrain;
    Sorter sorter;
    Shooter2 shooter;
    Intake intake;
    ArtifactSystem artifactSystem;
    private Follower follower;

    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime stateTimer = new ElapsedTime();

    private Pose startPose;
    private Pose shootingPose;
    private Pose pos1;
    private Pose pos1Forward;
    private Pose pos2;
    private Pose pos2Forward;

    // One-shot command flags — cleared each loop
    private boolean dpadUp, dpadDown, dpadLeft, dpadRight;
    private boolean leftTrigger, rightTrigger;
    private boolean leftBumper, rightBumper;
    private boolean intakeToggle, servoTransfer, intakeReverse;
    private boolean buttonB;

    // Last-state for rising-edge detection
    private boolean lastButtonB, lastDpadUp, lastDpadDown, lastDpadLeft, lastDpadRight;
    private boolean lastLeftBumper, lastRightBumper;
    private boolean lastIntakeToggle, lastServoTransfer, lastIntakeReverse;

    private boolean isMoving = false;
    private PathChain currentPath = null;
    private boolean isfarshoot = false;

    private int currentCycle = 0;

    // stateStarted prevents re-entering the init block after arrival
    private boolean stateStarted = false;
    private int lastArtifactCount = 0;

    private boolean intakeSlowActive = false;
    private ElapsedTime intakeSlowTimer = new ElapsedTime();

    enum AutoState {
        INIT, GO_TO_SHOOTING_POS, GO_TO_COLLECTION,
        COLLECT_BALLS, RETURN_TO_SHOOT, RETURN_HOME, COMPLETE
    }

    AutoState currentState = AutoState.INIT;

    @Override
    public void runOpMode() {
        startPose    = getStartPose();
        shootingPose = getShootingPose();
        pos1         = getPos1();
        pos1Forward  = getPos1Forward();
        pos2         = getPos2();
        pos2Forward  = getPos2Forward();

        initializeRobot();

        telemetry.addData("Status", "Ready");
        telemetry.addData("Artifacts preloaded", artifactSystem.artifactCount);
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        runtime.reset();
        follower.setMaxPower(0.8);
        transitionToState(AutoState.GO_TO_SHOOTING_POS);
        currentCycle = 0;

        while (opModeIsActive()) {
            double currentTime = getRuntime();

            follower.update();
            sorter.update();
            shooter.setPIDFCoefficients();

            // Build rising edges
            boolean sendButtonB       = buttonB       && !lastButtonB;
            boolean sendDpadUp        = dpadUp        && !lastDpadUp;
            boolean sendDpadDown      = dpadDown      && !lastDpadDown;
            boolean sendDpadLeft      = dpadLeft      && !lastDpadLeft;
            boolean sendDpadRight     = dpadRight     && !lastDpadRight;
            boolean sendLeftBumper    = leftBumper    && !lastLeftBumper;
            boolean sendRightBumper   = rightBumper   && !lastRightBumper;
            boolean sendIntakeToggle  = intakeToggle  && !lastIntakeToggle;
            boolean sendServoTransfer = servoTransfer && !lastServoTransfer;
            boolean sendIntakeReverse = intakeReverse && !lastIntakeReverse;

            artifactSystem.update(
                    sendDpadUp, sendDpadDown, sendDpadLeft, sendDpadRight,
                    leftTrigger, rightTrigger,
                    sendLeftBumper, sendRightBumper,
                    sendIntakeToggle, sendServoTransfer, sendIntakeReverse,
                    sendButtonB, currentTime);
            // Detect new artifact collected
            if (artifactSystem.artifactCount > lastArtifactCount) {
                intakeSlowActive = true;
                intakeSlowTimer.reset();
            }

            lastArtifactCount = artifactSystem.artifactCount;
            if (intakeSlowActive) {
                intake.forwardPower = 0.6 * (12.8 / hw.getBatteryVoltage());

                if (intakeSlowTimer.seconds() > 0.5) {
                    intakeSlowActive = false;
                    intake.forwardPower = 0.8 * (12.8 / hw.getBatteryVoltage());
                }
            }

            // Save for next loop
            lastButtonB       = buttonB;
            lastDpadUp        = dpadUp;
            lastDpadDown      = dpadDown;
            lastDpadLeft      = dpadLeft;
            lastDpadRight     = dpadRight;
            lastLeftBumper    = leftBumper;
            lastRightBumper   = rightBumper;
            lastIntakeToggle  = intakeToggle;
            lastServoTransfer = servoTransfer;
            lastIntakeReverse = intakeReverse;

            // Clear one-shot flags
            dpadUp = dpadDown = dpadLeft = dpadRight = false;
            leftBumper = rightBumper = false;
            intakeToggle = servoTransfer = intakeReverse = buttonB = false;

            switch (currentState) {
                case GO_TO_SHOOTING_POS: runGoToShootingPos(); break;
                case GO_TO_COLLECTION:   runGoToCollection();  break;
                case COLLECT_BALLS:      runCollectBalls();    break;
                case RETURN_TO_SHOOT:    runReturnToShoot();   break;
                case RETURN_HOME:        runReturnHome();      break;
                case COMPLETE:           runComplete(); return;
            }

            updateTelemetry();
            //sleep(10);
        }
    }

    private void initializeRobot() {
        hw = new RobotHardware();
        hw.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        drivetrain    = new Drivetrain(hw, follower, false);
        intake        = new Intake(hw, false);
        sorter        = new Sorter(hw, false);
        shooter       = new Shooter2(hw, false);
        artifactSystem = new ArtifactSystem(hw, telemetry, sorter, shooter, intake, false);

        artifactSystem.storedArtifacts[0] = "G";
        artifactSystem.storedArtifacts[1] = "P";
        artifactSystem.storedArtifacts[2] = "P";
        artifactSystem.artifactCount  = 3;
        artifactSystem.nextSlotIndex  = 0;
        artifactSystem.currentSlot    = 0;
        artifactSystem.offSetApplied  = false;
    }

    // ===================== PEDRO PATHING =====================

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
            default: // "linear"
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

    private void goToPose(Pose targetPose, String interpolationType) {
        goToPosition(targetPose.getX(), targetPose.getY(),
                Math.toDegrees(targetPose.getHeading()), interpolationType);
    }

    private boolean hasReachedTarget() {
        if (!isMoving) return true;
        if (!follower.isBusy()) {
            isMoving    = false;
            currentPath = null;
            return true;
        }
        return false;
    }

    // ===================== STATE FUNCTIONS =====================

    private void runGoToShootingPos() {
        if (!stateStarted) {
            stateStarted = true;
            follower.setMaxPower(0.8);
            artifactSystem.artifactCount = 3;
            hw.reverseauton = false;
            intake.stop();
            goToPose(shootingPose, "linear");
            dpadUp = true; // enter SHOOT state
        }

        boolean arrived = hasReachedTarget();

        if (arrived) {
            //artifactSystem.shootphase1 = 0.25;
            //artifactSystem.shootphase2 = 0.3;
            buttonB = true; // auto-fire — ArtifactSystem ignores repeated calls while already shooting
        }

        if (arrived && !artifactSystem.isActivelyShooting() && artifactSystem.artifactCount == 0) {
            if      (currentCycle == 0) { currentCycle = 1; transitionToState(AutoState.GO_TO_COLLECTION); }
            else if (currentCycle == 1) { currentCycle = 2; transitionToState(AutoState.GO_TO_COLLECTION); }
            else                        { transitionToState(AutoState.RETURN_HOME); }
        }

        if (stateTimer.seconds() > 20.0) {
            isMoving = false;
            transitionToState(currentCycle < 2 ? AutoState.GO_TO_COLLECTION : AutoState.RETURN_HOME);
        }
    }

    private void runGoToCollection() {
        if (!stateStarted) {
            stateStarted = true;
            dpadRight = true; // exit SHOOT → INTAKE
            if      (currentCycle == 1) goToPose(pos1, "linear");
            else if (currentCycle == 2) goToPose(pos2, "linear");
        }

        if (hasReachedTarget()) {
            intake.forwardPower = 1.0;
            intakeToggle = true;
            transitionToState(AutoState.COLLECT_BALLS);
        }

        if (stateTimer.seconds() > 5.0) {
            isMoving = false;
            transitionToState(AutoState.COLLECT_BALLS);
        }
    }

    private void runCollectBalls() {
        if (!stateStarted) {
            stateStarted = true;
            follower.setMaxPower(0.25 * (12.8 / hw.getBatteryVoltage()));
            if      (currentCycle == 1) goToPose(pos1Forward, "linear");
            else if (currentCycle == 2) goToPose(pos2Forward, "linear");
        } else if (artifactSystem.artifactCount >= 3
                || hasReachedTarget()
                || stateTimer.seconds() > 6.0) {

            transitionToState(AutoState.RETURN_TO_SHOOT);
        }

    }

    private void runReturnToShoot() {
        if (!stateStarted) {
            stateStarted = true;
            //intakeToggle = true; // turn intake off
            //intake.stop();
            //hw.reverseauton = true;
            //intakeReverse = true;
            follower.setMaxPower(1);
            goToPose(shootingPose, "linear");
            dpadUp = true; // re-enter SHOOT state
        }

        if (hasReachedTarget()) {
            transitionToState(AutoState.GO_TO_SHOOTING_POS);
        }

        if (stateTimer.seconds() > 5.0) {
            isMoving = false;
            transitionToState(AutoState.GO_TO_SHOOTING_POS);
        }
    }

    private void runReturnHome() {
        if (!stateStarted) {
            stateStarted = true;
            dpadRight = true;
            follower.setMaxPower(1.0);
            goToPose(shootingPose, "constant");
        }

        if (hasReachedTarget()) {
            transitionToState(AutoState.COMPLETE);
        }

        if (stateTimer.seconds() > 5.0) {
            isMoving = false;
            transitionToState(AutoState.COMPLETE);
        }
    }

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
        stateStarted = false; // ← reset so new state's init block runs exactly once
        stateTimer.reset();
    }

    private void updateTelemetry() {
        telemetry.addData("State",       currentState);
        telemetry.addData("Cycle",       currentCycle + " (0=Init, 1=Pos1, 2=Pos2)");
        telemetry.addData("Runtime",     "%.1f s", runtime.seconds());
        telemetry.addData("State Time",  "%.1f s", stateTimer.seconds());
        telemetry.addData("X",           "%.1f in", follower.getPose().getX());
        telemetry.addData("Y",           "%.1f in", follower.getPose().getY());
        telemetry.addData("Heading",     "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path Busy",   follower.isBusy());
        telemetry.addData("Artifacts",   artifactSystem.artifactCount);
        telemetry.addData("Robot State", artifactSystem.robotState);
        telemetry.addData("Shoot State", artifactSystem.shootSubState);
        telemetry.addData("Shooter RPM", shooter.currentVelocity());
        telemetry.update();
    }
}