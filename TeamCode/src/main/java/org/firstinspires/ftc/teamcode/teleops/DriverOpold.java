package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Sorterold;
import org.firstinspires.ftc.teamcode.mathUtils.ShooterCalculator;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArtifactSystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter2;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;

import java.util.List;
@Disabled
@Configurable
@TeleOp(name = "DriverOpold")
public class DriverOpold extends LinearOpMode {

    // ── Shared subsystems ──────────────────────────────────────────────────────
    RobotHardware     hw;
    Follower          follower;
    Drivetrain        drivetrain;
    Sorter            sorter;
    Shooter2          shooter;
    Intake            intake;
    ArtifactSystem    artifactSystem;
    ShooterCalculator calc;

    PanelsTelemetry tel = PanelsTelemetry.INSTANCE;

    // ── DriverOp tables ───────────────────────────────────────────────────────
    double[] distanceTable = {24, 48, 72, 144};
    double[] rpmTable      = {1650, 1850, 2050, 2650};

    // ── ManualOp state ────────────────────────────────────────────────────────
    public static int ShooterFarRPM   = 2500;
    public static int ShooterCloseRPM = 2000;

    private boolean lastIntakeButton = false;
    private boolean intakeOn         = false;
    private boolean reversing        = false;
    private double  reverseTime      = 0;
    private int     transferState    = 0;
    private boolean lastY, lastX, lastB, lastLB, lastRB, lastLT, lastRT;
    private boolean lastDpadUp, lastDpadDown, lastDpadLeft, lastDpadRight;

    // ── Mode switching ────────────────────────────────────────────────────────
    private enum Mode { DRIVER, MANUAL }
    private Mode    mode           = Mode.DRIVER;
    private boolean lastL3         = false;
    private boolean lastR3         = false;

    // ─────────────────────────────────────────────────────────────────────────

    public List<LynxModule> allHubs;
    ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        hw = new RobotHardware();
        hw.init(hardwareMap);

        follower   = Constants.createFollower(hardwareMap);
        drivetrain = new Drivetrain(hw, follower, false);
        intake     = new Intake(hw, false);
        sorter     = new Sorter(hw, intake, false);
        shooter    = new Shooter2(hw, false);
        artifactSystem = new ArtifactSystem(hw, telemetry, sorter, shooter, intake, false);
        calc       = new ShooterCalculator(distanceTable, rpmTable);
        follower.setPose(new Pose(-35, 0, 0));
        shooter.setTargetVelRPM(0);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        waitForStart();

        while (opModeIsActive()) {
            allHubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
            double currentTime = getRuntime();

            // ── Mode toggle (gamepad1 L3 → MANUAL, R3 → DRIVER) ──────────────
            boolean l3 = gamepad2.left_stick_button;
            boolean r3 = gamepad2.right_stick_button;

            if (l3 && !lastL3) {
                mode = Mode.MANUAL;
                telemetry.addData("[MODE]", "MANUAL (backup)");
            }
            if (r3 && !lastR3) {
                mode = Mode.DRIVER;
                telemetry.addData("[MODE]", "DRIVER (auto)");
            }
            lastL3 = l3;
            lastR3 = r3;

            double loopTime = loopTimer.milliseconds();

            telemetry.addData("Mode", mode);
            telemetry.addData("Loop Time:", loopTime);
            telemetry.update();
            loopTimer.reset();

            // ── Shared drivetrain inputs ──────────────────────────────────────
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            if (mode == Mode.DRIVER) {
                runDriverMode(y, x, rx, currentTime);
            } else {
                runManualMode(y, x, rx, currentTime);
            }
            if (gamepad2.dpad_right) {
                sorter.resetPID();
            }
            if (gamepad2.touchpad) {
                sorter.moveDegrees(55);
                sleep(100);

            }
        }

        // ── Cleanup ───────────────────────────────────────────────────────────
        drivetrain.stop();
        sorter.resetPID();
        artifactSystem.resetSlot();
        intake.stop();
    }

    // ══════════════════════════════════════════════════════════════════════════
    //  DRIVER MODE  (original DriverOp logic)
    // ══════════════════════════════════════════════════════════════════════════
    private void runDriverMode(double y, double x, double rx, double currentTime) {
        shooter.setPIDFCoefficients();

        double rotInput = rx;
        if (gamepad1.a) rotInput = drivetrain.facePoint(8, 0);

        if (gamepad1.b) {
            follower.setPose(new Pose(0, 0, 0));
        }
        drivetrain.drive(y, x, rotInput);

        Pose currentPose = follower.getPose();
        double dx = 0 - currentPose.getX();
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

        if (rightEdge) artifactSystem.switchToIntake();
        if (leftEdge) artifactSystem.switchToShooting(ArtifactSystem.SHORT_RPM, true);
        if (upEdge) artifactSystem.switchToShooting(ArtifactSystem.LONG_RPM, false);
        if (downEdge) artifactSystem.switchToShooting(ArtifactSystem.SHORT_RPM, false);

        if (yEdge) artifactSystem.toggleIntake();

        if (artifactSystem.robotState == ArtifactSystem.RobotState.INTAKE) {
            artifactSystem.setIntakeReverse(gamepad2.a, currentTime);
            if (lbEdge) artifactSystem.manualDetect("G");
            if (rbEdge) artifactSystem.manualDetect("P");
            if (ltEdge) artifactSystem.inspectSlot("G");
            if (rtEdge) artifactSystem.inspectSlot("P");
        } else if (artifactSystem.robotState == ArtifactSystem.RobotState.SHOOTING) {
            if (bEdge) artifactSystem.triggerAutoFire();
            if (xEdge) artifactSystem.triggerManualShot(currentTime);
            if (lbEdge) artifactSystem.triggerColorShot("G");
            if (rbEdge) artifactSystem.triggerColorShot("P");
        }

        artifactSystem.update(currentTime);

        follower.setMaxPower(1);
        sorter.update();
        follower.update();
        drivetrain.telemetryOn = false;

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
    }

    // ══════════════════════════════════════════════════════════════════════════
    //  MANUAL MODE  (original ManualOp logic)
    // ══════════════════════════════════════════════════════════════════════════
    private void runManualMode(double y, double x, double rx, double currentTime) {
        manualDrivetrain(y, x, rx);
        manualIntake(gamepad2.y, gamepad2.a, currentTime);
        manualSorter();
        manualTransfer();
        manualShooter();
        updateColorDetection();
    }

    private void manualDrivetrain(double y, double x, double rx) {
        hw.leftFront.setPower( y + x + rx);
        hw.leftBack.setPower(  y - x + rx);
        hw.rightFront.setPower(y - x - rx);
        hw.rightBack.setPower( y + x - rx);
    }

    private void manualShooter() {
        if      (gamepad2.dpad_up)   shooter.setTargetVelRPM(ShooterFarRPM);
        else if (gamepad2.dpad_left) shooter.setTargetVelRPM(ShooterCloseRPM);
        else if (gamepad2.dpad_down) shooter.setTargetVelRPM(0);
    }

    private void manualIntake(boolean button, boolean reverseButton, double currentTime) {
        if (button && !lastIntakeButton) intakeOn = !intakeOn;

        if (reverseButton) {
            reversing   = true;
            reverseTime = currentTime + 0.5;
        }

        if (reversing) {
            hw.intake.setPower(-1);
            if (!reverseButton && currentTime >= reverseTime) reversing = false;
        } else {
            hw.intake.setPower(intakeOn ? 1 : 0);
        }

        if (intakeOn) shooter.setTargetVelRPM(100);

        lastIntakeButton = button;
    }

    private void manualTransfer() {
        switch (transferState) {
            case 0:
                if (gamepad2.x) {
                    hw.sorterTransfer.setPosition(RobotHardware.transferPush);
                    hw.timer.reset();
                    transferState = 1;
                }
                break;
            case 1:
                if (hw.timer.seconds() >= 0.3) {
                    hw.sorterTransfer.setPosition(RobotHardware.transferIdle);
                    hw.timer.reset();
                    transferState = 0;
                }
                break;
        }
    }

    private void manualSorter() {
        hw.sorter.setPower((gamepad2.left_trigger - gamepad2.right_trigger) / 5);
    }

    private void updateColorDetection() {
        int red   = hw.colorSensor.red();
        int green = hw.colorSensor.green();
        int blue  = hw.colorSensor.blue();
        int alpha = hw.colorSensor.alpha();

        boolean artifactDetected = alpha > 150 && !(blue > red && blue > green);

    }
}
