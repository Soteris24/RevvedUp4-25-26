package org.firstinspires.ftc.teamcode.autonomous;

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

public abstract class BaseAutonomous1 extends LinearOpMode {
    protected abstract Pose getStartPose();
    protected abstract Pose getShootingPose();
    protected abstract Pose getPos1();
    protected abstract Pose getPos1Forward();
    protected abstract Pose getPos2();
    protected abstract Pose getPos2Forward();

    protected RobotHardware hw;
    protected Drivetrain drivetrain;
    protected Sorter sorter;
    protected Shooter2 shooter;
    protected Intake intake;
    protected ArtifactSystem artifactSystem;
    protected Follower follower;

    protected ElapsedTime runtime = new ElapsedTime();
    protected ElapsedTime stateTimer = new ElapsedTime();

    protected Pose startPose, shootingPose, pos1, pos1Forward, pos2, pos2Forward;
    protected int currentCycle = 0;
    protected boolean stateStarted = false;
    protected boolean isMoving = false;

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

        waitForStart();
        if (!opModeIsActive()) return;

        runtime.reset();
        transitionToState(AutoState.GO_TO_SHOOTING_POS);

        while (opModeIsActive()) {
            double currentTime = getRuntime();

            follower.update();
            sorter.update();
            shooter.setPIDFCoefficients();
            artifactSystem.update(currentTime, false, false, false);

            //  Safety
            if (runtime.seconds() > 28.5 && currentState != AutoState.RETURN_HOME && currentState != AutoState.COMPLETE) {
                transitionToState(AutoState.RETURN_HOME);
            }

            switch (currentState) {
                case GO_TO_SHOOTING_POS: runGoToShootingPos(); break;
                case GO_TO_COLLECTION:   runGoToCollection();  break;
                case COLLECT_BALLS:      runCollectBalls();    break;
                case RETURN_TO_SHOOT:    runReturnToShoot(currentTime); break;
                case RETURN_HOME:        runReturnHome();      break;
                case COMPLETE:           runComplete(); return;
            }

            updateTelemetry();
        }
    }

    protected void initializeRobot() {
        hw = new RobotHardware();
        hw.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        drivetrain     = new Drivetrain(hw, follower, false);
        intake         = new Intake(hw, false);
        sorter         = new Sorter(hw, false);
        shooter        = new Shooter2(hw, false);
        artifactSystem = new ArtifactSystem(hw, telemetry, sorter, shooter, intake, false);

        // Preload logic
        artifactSystem.manualDetect("G");
        artifactSystem.manualDetect("P");
        artifactSystem.manualDetect("P");
        artifactSystem.artifactCount = 3;
    }

    protected void goToPose(Pose targetPose, String interpolationType) {
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), targetPose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPose.getHeading())
                .build();
        follower.followPath(path);
        isMoving = true;
    }

    protected boolean hasReachedTarget() {
        if (!isMoving) return true;
        if (!follower.isBusy()) { isMoving = false; return true; }
        return false;
    }

    protected void transitionToState(AutoState newState) {
        currentState = newState;
        stateStarted = false;
        stateTimer.reset();
    }

    private void runGoToShootingPos() {
        if (!stateStarted) {
            stateStarted = true;
            follower.setMaxPower(0.8);
            goToPose(shootingPose, "linear");
            artifactSystem.switchToShooting(ArtifactSystem.SHORT_RPM, false);
        }

        if (hasReachedTarget()) {
            artifactSystem.triggerAutoFire();
            if (!artifactSystem.isActivelyShooting() && artifactSystem.artifactCount == 0) {
                if (currentCycle < 2) {
                    currentCycle++;
                    transitionToState(AutoState.GO_TO_COLLECTION);
                } else {
                    transitionToState(AutoState.RETURN_HOME);
                }
            }
        }
    }

    private void runGoToCollection() {
        if (!stateStarted) {
            stateStarted = true;
            artifactSystem.switchToIntake();
            goToPose(currentCycle == 1 ? pos1 : pos2, "linear");
        }

        if (hasReachedTarget()) {
            intake.forwardPower = 1.0;
            artifactSystem.toggleIntake();
            transitionToState(AutoState.COLLECT_BALLS);
        }
    }

    private void runCollectBalls() {
        if (!stateStarted) {
            stateStarted = true;
            // Preserved "Far" side specific power adjustment
            follower.setMaxPower(0.25);
            goToPose(currentCycle == 1 ? pos1Forward : pos2Forward, "linear");
        } else if (artifactSystem.artifactCount >= 3 || hasReachedTarget() || stateTimer.seconds() > 6.0) {
            transitionToState(AutoState.RETURN_TO_SHOOT);
        }
    }

    private void runReturnToShoot(double currentTime) {
        if (!stateStarted) {
            stateStarted = true;
            follower.setMaxPower(0.8);
            goToPose(shootingPose, "linear");
        }

        // Intake Clearing Sequence:
        // 0.0s - 1.0s: SUCK IN everything to seat it
        // 1.0s - 1.8s: REVERSE to clear ramp
        // 1.8s+: Stop intake and prepare shooter
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
    }

    private void runReturnHome() {
        if (!stateStarted) {
            stateStarted = true;
            artifactSystem.switchToIntake();
            follower.setMaxPower(1.0);
            goToPose(pos1, "linear"); // Far side returns to Pos1 (park)
        }
        if (hasReachedTarget()) transitionToState(AutoState.COMPLETE);
    }

    private void runComplete() {
        shooter.setTargetVelRPM(0);
        drivetrain.stop();
        artifactSystem.resetSlot();
    }

    private void updateTelemetry() {
        telemetry.addData("State", currentState);
        telemetry.addData("Cycle", currentCycle);
        telemetry.addData("Artifacts", artifactSystem.artifactCount);
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}
