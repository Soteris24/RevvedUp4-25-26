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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArtifactSystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter2;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;

import java.util.List;

@Configurable
@TeleOp(name = "DriverOp")
public class DriverOp extends LinearOpMode {

    RobotHardware     hw;
    Follower          follower;
    Drivetrain        drivetrain;
    Sorter            sorter;
    Shooter2          shooter;
    Intake            intake;
    ArtifactSystem    artifactSystem;

    PanelsTelemetry tel = PanelsTelemetry.INSTANCE;
    ElapsedTime loopTimer = new ElapsedTime();
    List<LynxModule> allHubs;

    // Edge detection — one canonical set, never duplicated
    boolean lastY, lastX, lastB, lastRB, lastLB, lastLT, lastRT;
    boolean lastDpadUp, lastDpadDown, lastDpadLeft, lastDpadRight;

    @Override
    public void runOpMode() {
        hw = new RobotHardware();
        hw.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        drivetrain     = new Drivetrain(hw, follower, true);
        intake         = new Intake(hw, false);
        sorter         = new Sorter(hw, false);
        shooter        = new Shooter2(hw, false);
        artifactSystem = new ArtifactSystem(hw, telemetry, sorter, shooter, intake, true);

        // Lynx bulk caching — set once before loop
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower.setPose(new Pose(-35, 0, 0));
        shooter.setTargetVelRPM(0);

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {

            // Clear bulk cache once per loop (MANUAL mode)
            for (LynxModule hub : allHubs) hub.clearBulkCache();

            double currentTime = getRuntime();
            shooter.setPIDFCoefficients();

            // ── Telemetry ──────────────────────────────────────────────────
            telemetry.addData("Loop ms", loopTimer.milliseconds());
            loopTimer.reset();

            // ── Drivetrain (Gamepad 1) ─────────────────────────────────────
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;
            double rotInput = rx;

            if (gamepad1.a) rotInput = drivetrain.facePoint(8, 0);
            if (gamepad1.b) follower.setPose(new Pose(0, 0, 0));
            drivetrain.drive(y, x, rotInput);

            Pose p = follower.getPose();
            artifactSystem.currentDistance = Math.hypot(p.getX(), p.getY());

            // ── Compute ALL edges once, here, never again ──────────────────
            boolean upEdge    = gamepad2.dpad_up     && !lastDpadUp;
            boolean downEdge  = gamepad2.dpad_down   && !lastDpadDown;
            boolean leftEdge  = gamepad2.dpad_left   && !lastDpadLeft;
            boolean rightEdge = gamepad2.dpad_right  && !lastDpadRight;
            boolean yEdge     = gamepad2.y            && !lastY;
            boolean bEdge     = gamepad2.b            && !lastB;
            boolean xEdge     = gamepad2.x            && !lastX;
            boolean lbEdge    = gamepad2.left_bumper  && !lastLB;
            boolean rbEdge    = gamepad2.right_bumper && !lastRB;
            boolean ltEdge    = (gamepad2.left_trigger  > 0.5) && !lastLT;
            boolean rtEdge    = (gamepad2.right_trigger > 0.5) && !lastRT;

            // ── Mode switching (edge only) ─────────────────────────────────
            if (rightEdge) artifactSystem.switchToIntake();
            if (leftEdge)  artifactSystem.switchToShooting(ArtifactSystem.SHORT_RPM, true);
            if (upEdge)    artifactSystem.switchToShooting(ArtifactSystem.LONG_RPM, false);
            if (downEdge)  artifactSystem.switchToShooting(ArtifactSystem.SHORT_RPM, false);

            // ── Intake (edge only, no unconditional calls) ─────────────────
            if (yEdge) artifactSystem.toggleIntake();

            // Reverse: only pass through to ArtifactSystem when in INTAKE state
            // to avoid fighting intake.stop() in other states
            if (artifactSystem.robotState == ArtifactSystem.RobotState.INTAKE) {
                artifactSystem.setIntakeReverse(gamepad2.a, currentTime);
            }

            // ── Shooting (edge only) ───────────────────────────────────────
            if (bEdge)  artifactSystem.triggerAutoFire();
            if (xEdge)  artifactSystem.triggerManualShot(currentTime);
            if (lbEdge) artifactSystem.triggerColorShot("G");
            if (rbEdge) artifactSystem.triggerColorShot("P");

            // ── Manual detect (edge only) ──────────────────────────────────
            if (ltEdge) artifactSystem.manualDetect("P");
            if (rtEdge) artifactSystem.manualDetect("G");

            // ── System update — pass edges, NOT raw buttons ────────────────
            // ltEdge/rtEdge here are for MANUAL state sorter rotation,
            // xEdge is NOT passed as servoEdge — triggerManualShot handles that
            artifactSystem.update(currentTime, ltEdge, rtEdge, false);

            sorter.update();
            follower.update();
            follower.setMaxPower(1);
            drivetrain.telemetryOn = false;

            // ── Save history — must be last ────────────────────────────────
            lastY  = gamepad2.y;
            lastX  = gamepad2.x;
            lastB  = gamepad2.b;
            lastLB = gamepad2.left_bumper;
            lastRB = gamepad2.right_bumper;
            lastDpadUp    = gamepad2.dpad_up;
            lastDpadDown  = gamepad2.dpad_down;
            lastDpadLeft  = gamepad2.dpad_left;
            lastDpadRight = gamepad2.dpad_right;
            lastLT = (gamepad2.left_trigger  > 0.5);
            lastRT = (gamepad2.right_trigger > 0.5);

            telemetry.update();
        }

        drivetrain.stop();
        sorter.resetPID();
        artifactSystem.resetSlot();
        intake.stop();
    }
}