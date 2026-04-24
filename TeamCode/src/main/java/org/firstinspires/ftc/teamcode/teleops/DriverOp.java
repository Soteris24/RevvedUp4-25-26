package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.mathUtils.ShooterCalculator;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArtifactSystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LimelightPoseCorrector;
import org.firstinspires.ftc.teamcode.subsystems.Shooter2;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;

import java.util.List;
@Configurable
@TeleOp(name = "DriverOp")
public class DriverOp extends LinearOpMode {

    RobotHardware     hw;
    Follower         follower;
    Drivetrain        drivetrain;
    Sorter            sorter;
    Shooter2          shooter;
    Intake            intake;
    ArtifactSystem    artifactSystem;
    ShooterCalculator calc;
    LimelightPoseCorrector limelightPoseCorrector;

    double[] distanceTable = {24, 48, 72, 144};
    double[] rpmTable      = {1650, 1850, 2050, 2650};

    PanelsTelemetry tel = PanelsTelemetry.INSTANCE;
    ElapsedTime loopTimer = new ElapsedTime();
    List<LynxModule> allHubs;

    boolean lastY;
    boolean lastX;
    boolean lastB;
    boolean lastLB;
    boolean lastRB;
    boolean lastLT;
    boolean lastRT;
    boolean lastDpadUp;
    boolean lastDpadDown;
    boolean lastDpadLeft;
    boolean lastDpadRight;

    @Override
    public void runOpMode() {
        hw = new RobotHardware();
        hw.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);

        drivetrain = new Drivetrain(hw, follower, true);
        intake         = new Intake(hw, false);
        sorter         = new Sorter(hw, intake, false);
        shooter        = new Shooter2(hw, false);
        artifactSystem = new ArtifactSystem(hw, telemetry, sorter, shooter, intake, true);
        calc           = new ShooterCalculator(distanceTable, rpmTable);
        limelightPoseCorrector = new LimelightPoseCorrector(hardwareMap, follower, telemetry);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();
        limelightPoseCorrector.start();
        loopTimer.reset();

        while (opModeIsActive()) {
            double currentTime = getRuntime();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            shooter.setPIDFCoefficients();
            telemetry.addData("Loop ms", loopTimer.milliseconds());
            loopTimer.reset();

            follower.setMaxPower(1);
            follower.update();
            limelightPoseCorrector.update();

            // --- Drivetrain (gamepad1) ---
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            double rotInput = rx;

            if (gamepad1.a) {
                rotInput = drivetrain.faceGoal();
            }
            if (!gamepad1.a) {
                drivetrain.resetAimController();
            }

            drivetrain.drive(y, x, rotInput);


            Pose currentPose = follower.getPose();
            double dx = 0 - currentPose.getX(); // goal coordinates
            double dy = 0 - currentPose.getY();
            artifactSystem.currentDistance = Math.hypot(dx, dy);

            boolean upEdge = gamepad2.dpad_up && !lastDpadUp;
            boolean downEdge = gamepad2.dpad_down && !lastDpadDown;
            boolean leftEdge = gamepad2.dpad_left && !lastDpadLeft;
            boolean rightEdge = gamepad2.dpad_right && !lastDpadRight;
            boolean yEdge = gamepad2.y && !lastY;
            boolean xEdge = gamepad2.x && !lastX;
            boolean bEdge = gamepad2.b && !lastB;
            boolean lbEdge = gamepad2.left_bumper && !lastLB;
            boolean rbEdge = gamepad2.right_bumper && !lastRB;
            boolean ltEdge = gamepad2.left_trigger > 0.5 && !lastLT;
            boolean rtEdge = gamepad2.right_trigger > 0.5 && !lastRT;

            if (rightEdge) {
                artifactSystem.switchToIntake();
            }
            if (leftEdge) {
                artifactSystem.switchToShooting(ArtifactSystem.SHORT_RPM, true);
            }
            if (upEdge) {
                artifactSystem.switchToShooting(ArtifactSystem.LONG_RPM, false);
            }
            if (downEdge) {
                artifactSystem.switchToShooting(ArtifactSystem.SHORT_RPM, false);
            }

            if (yEdge) {
                artifactSystem.toggleIntake();
            }

            if (artifactSystem.robotState == ArtifactSystem.RobotState.INTAKE) {
                artifactSystem.setIntakeReverse(gamepad2.a, currentTime);
                if (lbEdge) {
                    artifactSystem.manualDetect("G");
                }
                if (rbEdge) {
                    artifactSystem.manualDetect("P");
                }
                if (ltEdge) {
                    artifactSystem.inspectSlot("G");
                }
                if (rtEdge) {
                    artifactSystem.inspectSlot("P");
                }
            } else if (artifactSystem.robotState == ArtifactSystem.RobotState.SHOOTING) {
                if (bEdge) {
                    artifactSystem.triggerAutoFire();
                }
                if (xEdge) {
                    artifactSystem.triggerManualShot(currentTime);
                }
                if (lbEdge) {
                    artifactSystem.triggerColorShot("G");
                }
                if (rbEdge) {
                    artifactSystem.triggerColorShot("P");
                }
            }

            artifactSystem.update(currentTime, ltEdge, rtEdge, false);

            sorter.update();
            drivetrain.telemetryOn = false;
            limelightPoseCorrector.addTelemetry();

            lastY = gamepad2.y;
            lastX = gamepad2.x;
            lastB = gamepad2.b;
            lastLB = gamepad2.left_bumper;
            lastRB = gamepad2.right_bumper;
            lastLT = gamepad2.left_trigger > 0.5;
            lastRT = gamepad2.right_trigger > 0.5;
            lastDpadUp = gamepad2.dpad_up;
            lastDpadDown = gamepad2.dpad_down;
            lastDpadLeft = gamepad2.dpad_left;
            lastDpadRight = gamepad2.dpad_right;

            telemetry.update();
        }

        drivetrain.stop();
        limelightPoseCorrector.stop();
        sorter.resetPID();
        artifactSystem.resetSlot();
        intake.stop();
    }
}
