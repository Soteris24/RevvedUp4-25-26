package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.lynx.LynxModule;
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

import java.util.List;

public abstract class BaseAutonomous1 extends LinearOpMode {
    protected abstract Pose getStartPose();
    protected abstract Pose getShootingPose();
    protected abstract Pose getMotifPose();
    protected abstract Pose getPos1();
    protected abstract Pose getPos1Forward();
    protected abstract Pose getPos2();
    protected abstract Pose getPos2Forward();
    protected abstract Pose getHumanPlayer();
    protected abstract Pose getHumanPlayer2();
    protected abstract int getPipeline();
    RobotHardware hw;
    Drivetrain drivetrain;
    Sorter sorter;
    Shooter2 shooter;
    Intake intake;
    ArtifactSystem artifactSystem;
    private Follower follower;

    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime stateTimer = new ElapsedTime();
    List<LynxModule> allHubs;

    private Pose startPose;
    private Pose shootingPose;
    private Pose motifPose;
    private Pose pos1;
    private Pose pos1Forward;
    private Pose pos2;
    private Pose pos2Forward;
    private Pose humanPlayer;
    private Pose humanPlayer2;

    private int pipeline;

    private boolean isMoving = false;
    private PathChain currentPath = null;

    private int currentCycle = 0;

    // stateStarted prevents re-entering the init block after arrival
    private boolean stateStarted = false;
    private int lastArtifactCount = 0;

    private boolean intakeSlowActive = false;
    private boolean hasResumed = false;
    private ElapsedTime intakeSlowTimer = new ElapsedTime();

    enum AutoState {
        INIT, GO_TO_SHOOTING_POS, FACE_MOTIF, ALIGN_LIMELIGHT, GO_TO_COLLECTION, COLLECT_FROM_GATE,
        COLLECT_BALLS, RETURN_TO_SHOOT, RETURN_HOME, COMPLETE
    }

    AutoState currentState = AutoState.INIT;

    @Override
    public void runOpMode() {
        startPose    = getStartPose();
        shootingPose = getShootingPose();
        motifPose    = getMotifPose();
        pos1         = getPos1();
        pos1Forward  = getPos1Forward();
        pos2         = getPos2();
        pos2Forward  = getPos2Forward();
        humanPlayer  = getHumanPlayer();
        humanPlayer2  = getHumanPlayer2();
        pipeline     = getPipeline();

        initializeRobot();

//        telemetry.addData("Status", "Ready");
//        telemetry.addData("Artifacts preloaded", artifactSystem.artifactCount);
//        telemetry.update();
        sorter.moveDegrees(-ArtifactSystem.initialOffsets[0]);
        waitForStart();
        if (!opModeIsActive()) return;

        runtime.reset();
        follower.setMaxPower(0.8);
        transitionToState(AutoState.FACE_MOTIF);
        currentCycle = 0;

        while (opModeIsActive()) {

            if (runtime.seconds() > 29
                    && currentState != BaseAutonomous1.AutoState.RETURN_HOME
                    && currentState != BaseAutonomous1.AutoState.COMPLETE) {
                transitionToState(BaseAutonomous1.AutoState.RETURN_HOME);
            }

            double currentTime = getRuntime();

            follower.update();
            sorter.update();
            shooter.setPIDFCoefficients();
            intake.intake(false,false,currentTime);
            artifactSystem.update(currentTime);

//            if (!artifactSystem.motifSeen) {
//                artifactSystem.updateMotifFromAprilTag();
//            }

            // Detect new artifact collected
            if (artifactSystem.artifactCount > lastArtifactCount) {
                intakeSlowActive = true;
                intakeSlowTimer.reset();
            }

            lastArtifactCount = artifactSystem.artifactCount;
            switch (currentState) {
                case GO_TO_SHOOTING_POS: runGoToShootingPos(); break;
                case FACE_MOTIF:         runFaceMotif(); break;
                case ALIGN_LIMELIGHT:    runAlignLimelight(); break;
                case GO_TO_COLLECTION:   runGoToCollection();  break;
                case COLLECT_BALLS:      runCollectBalls();    break;
                case RETURN_TO_SHOOT:    runReturnToShoot(currentTime);   break;
                case RETURN_HOME:        runReturnHome();      break;
                case COMPLETE:           runComplete(); return;
            }

            //updateTelemetry();
            //sleep(10);
        }
    }

    private void initializeRobot() {
        hw = new RobotHardware();
        hw.init(hardwareMap, pipeline);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        drivetrain    = new Drivetrain(hw, follower, false);
        intake        = new Intake(hw, false);
        sorter        = new Sorter(hw, intake, false);
        shooter       = new Shooter2(hw, false);
        artifactSystem = new ArtifactSystem(hw, telemetry, sorter, shooter, intake, false);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
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
                        .setConstantHeadingInterpolation(targetPose.getHeading())
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
            artifactSystem.seedArtifacts("G", "P", "P");
            intake.stop();
            goToPose(shootingPose, "linear");
            artifactSystem.switchToShooting(ArtifactSystem.LONG_RPM, false);
        }

        boolean arrived = hasReachedTarget();

        if (arrived) {
            transitionToState(AutoState.ALIGN_LIMELIGHT);
        }

        if (stateTimer.seconds() > 20.0) {
            isMoving = false;
            transitionToState(AutoState.ALIGN_LIMELIGHT);
        }
    }

    private void runFaceMotif() {
        if (!stateStarted) {
            stateStarted = true;
            hw.pipelineSwitch(1);
            artifactSystem.motifSeen = false;
            goToPose(motifPose, "linear");
        }

        artifactSystem.updateMotifFromAprilTag();

        if ((artifactSystem.motifSeen && hasReachedTarget()) || stateTimer.seconds() > 2.0) {
            hw.pipelineSwitch(pipeline);
            transitionToState(AutoState.GO_TO_SHOOTING_POS);
        }
    }

    private void runAlignLimelight() {
        if (!stateStarted) {
            stateStarted = true;
            follower.breakFollowing();
        }

        double rotation = drivetrain.faceLimelightTarget();
        drivetrain.drive(0, 0, rotation);

        if (Math.abs(rotation) < 0.05 || stateTimer.seconds() > 1.0) {
            artifactSystem.triggerAutoMotifFire();

            // Check if done shooting
            if (!artifactSystem.isActivelyShooting() && artifactSystem.artifactCount == 0) {
                if      (currentCycle == 0) { currentCycle = 1; transitionToState(AutoState.GO_TO_COLLECTION); }
                else if (currentCycle == 1) { currentCycle = 2; transitionToState(AutoState.GO_TO_COLLECTION); }
                else                        { transitionToState(AutoState.RETURN_HOME); }
                artifactSystem.switchToIntake();
            }
        }
    }

    private void runGoToCollection() {
        if (!stateStarted) {
            stateStarted = true;
            artifactSystem.switchToIntake();
            intake.forwardPower = 1.0;
            follower.setMaxPower(1);
            artifactSystem.toggleIntake();
            if      (currentCycle == 1) goToPose(pos1, "linear");
            else if (currentCycle == 2) goToPose(humanPlayer, "constant");
        }

        if (hasReachedTarget()) {
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
            intakeSlowActive = false; // false = haven't paused for the 2nd ball yet
            hasResumed = false;
            follower.setMaxPower(0.6);
            if      (currentCycle == 1) goToPose(pos1Forward, "linear");
            else if (currentCycle == 2) goToPose(humanPlayer2, "constant");
        }

        // 1. Detect 2 balls and pause
        if (!intakeSlowActive && artifactSystem.artifactCount >= 1) {
            intakeSlowActive = true;
            follower.setMaxPower(0);
            intakeSlowTimer.reset();
        }

        // 2. After pause, speed up to 0.8 and resume movement
        if (intakeSlowActive && !hasResumed && intakeSlowTimer.seconds() > 0.5) {
            hasResumed = true;
            follower.setMaxPower(0.7);
            if      (currentCycle == 1) goToPose(pos1Forward, "linear");
            else if (currentCycle == 2) goToPose(humanPlayer2, "constant");
        }

        // 3. Exit condition (3 balls, target reached, or timeout)
        if (artifactSystem.artifactCount >= 3
                || (hasResumed && intakeSlowTimer.seconds() > 0.2 && hasReachedTarget())
                || stateTimer.seconds() > 5) {
                transitionToState(BaseAutonomous1.AutoState.RETURN_TO_SHOOT);
        }
    }

    private void runReturnToShoot(double currentTime) {
        if (!stateStarted) {
            stateStarted = true;
            follower.setMaxPower(1);
            goToPose(shootingPose, "constant");
        }

        if (stateTimer.seconds() < 1.0) {
            intake.intakeOn = true;
        } else if (stateTimer.seconds() < 1.8) {
            artifactSystem.setIntakeReverse(true, currentTime);
        } else {
            intake.stop();
            artifactSystem.switchToShooting(ArtifactSystem.LONG_RPM, false);
        }

        if (hasReachedTarget() && stateTimer.seconds() > 0) {
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
            sorter.moveDegrees(ArtifactSystem.initialOffsets[0]);
            follower.setMaxPower(1.0);
            goToPose(pos1, "linear");
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
    }

    private void transitionToState(AutoState newState) {
        currentState = newState;
        stateStarted = false; // ← reset so new state's init block runs exactly once
        stateTimer.reset();
    }

    private void updateTelemetry() {
//        telemetry.addData("State",       currentState);
//        telemetry.addData("Cycle",       currentCycle + " (0=Init, 1=Pos1, 2=Pos2)");
//        telemetry.addData("Runtime",     "%.1f s", runtime.seconds());
//        telemetry.addData("State Time",  "%.1f s", stateTimer.seconds());
//        telemetry.addData("X",           "%.1f in", follower.getPose().getX());
//        telemetry.addData("Y",           "%.1f in", follower.getPose().getY());
//        telemetry.addData("Heading",     "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
//        telemetry.addData("Path Busy",   follower.isBusy());
//        telemetry.addData("Artifacts",   artifactSystem.artifactCount);
//        telemetry.addData("Robot State", artifactSystem.robotState);
//        telemetry.addData("Shoot State", artifactSystem.shootSubState);
//        telemetry.addData("Shooter RPM", shooter.currentVelocity());
//        telemetry.update();
    }
}
