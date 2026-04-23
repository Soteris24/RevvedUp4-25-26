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

public abstract class BaseAutonomous extends LinearOpMode {
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

    private boolean isMoving = false;
    private PathChain currentPath = null;

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

            artifactSystem.update(currentTime, false, false, false);

            if (runtime.seconds() > 28.5
                    && currentState != AutoState.RETURN_HOME
                    && currentState != AutoState.COMPLETE) {
                transitionToState(AutoState.RETURN_HOME);
            }

            switch (currentState) {
                case GO_TO_SHOOTING_POS: runGoToShootingPos(); break;
                case GO_TO_COLLECTION:   runGoToCollection();  break;
                case COLLECT_BALLS:      runCollectBalls();    break;
                case RETURN_TO_SHOOT:    runReturnToShoot(currentTime);   break;
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

        artifactSystem.seedArtifacts("G", "P", "P");
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
            hw.reverseauton = false;
            intake.stop();
            goToPose(shootingPose, "linear");
            artifactSystem.switchToShooting(ArtifactSystem.SHORT_RPM, false);
        }

        boolean arrived = hasReachedTarget();

        if (arrived) {
            artifactSystem.triggerAutoFire();
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
            artifactSystem.switchToIntake();
            if      (currentCycle == 1) goToPose(pos1, "linear");
            else if (currentCycle == 2) goToPose(pos2, "linear");
        }

        if (hasReachedTarget()) {
            intake.forwardPower = 1.0;
            follower.setMaxPower(1);
            artifactSystem.toggleIntake();
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
            follower.setMaxPower(0.26 * (12.8 / hw.getBatteryVoltage() / 1.05)); //
            if      (currentCycle == 1) goToPose(pos1Forward, "linear");
            else if (currentCycle == 2) goToPose(pos2Forward, "linear");
        } else if (artifactSystem.artifactCount >= 3
                || hasReachedTarget()
                || stateTimer.seconds() > 6.0) {

            transitionToState(AutoState.RETURN_TO_SHOOT);
        }

    }

    private void runReturnToShoot(double currentTime) {
        if (!stateStarted) {
            stateStarted = true;
            follower.setMaxPower(1);
            goToPose(shootingPose, "linear");
        }

        if (stateTimer.seconds() < 1.0) {
            intake.intakeOn = true;
        } else if (stateTimer.seconds() < 1.8) {
            artifactSystem.setIntakeReverse(true, currentTime);
        } else {
            intake.stop();
            artifactSystem.switchToShooting(ArtifactSystem.SHORT_RPM, false);
        }

        if (hasReachedTarget() && stateTimer.seconds() > 2.0) {
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
            artifactSystem.switchToIntake();
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
