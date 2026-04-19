package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArtifactSystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter2;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;

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

    // Edge detection
    boolean lastY, lastX, lastB, lastRB, lastLB, lastRT, lastLT;
    boolean lastDpadUp, lastDpadDown, lastDpadLeft, lastDpadRight;

    @Override
    public void runOpMode() {
        hw = new RobotHardware();
        hw.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        drivetrain     = new Drivetrain(hw, follower, true);
        intake         = new Intake(hw, false);
        sorter         = new Sorter(hw, false, intake);
        shooter        = new Shooter2(hw, false);
        artifactSystem = new ArtifactSystem(hw, telemetry, sorter, shooter, intake, true);

        waitForStart();

        while (opModeIsActive()) {
            double currentTime = getRuntime();
            shooter.setPIDFCoefficients();

            // --- DRIVETRAIN (Gamepad 1) ---
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;
            double rotInput = rx;

            if (gamepad1.a) rotInput = drivetrain.facePoint(40, 0);
            if (gamepad1.b) rotInput = drivetrain.facePoint(0, 0);
            drivetrain.drive(y, x, rotInput);

            // Update shooting distance based on pose
            Pose p = follower.getPose();
            artifactSystem.currentDistance = Math.hypot(p.getX(), p.getY());

            // --- ARTIFACT SYSTEM (Gamepad 2) ---
            
            // Mode Switching
            if (gamepad2.dpad_right) artifactSystem.switchToIntake();
            if (gamepad2.dpad_left)  artifactSystem.switchToShooting(ArtifactSystem.SHORT_RPM, true);
            if (gamepad2.dpad_up)    artifactSystem.switchToShooting(ArtifactSystem.LONG_RPM, false);
            if (gamepad2.dpad_down)  artifactSystem.switchToShooting(ArtifactSystem.SHORT_RPM, false);

            // Intake Controls
            if (gamepad2.y && !lastY) artifactSystem.toggleIntake();
            artifactSystem.setIntakeReverse(gamepad2.a, currentTime);

            // Shooting Controls
            if (gamepad2.b && !lastB)  artifactSystem.triggerAutoFire();
            if (gamepad2.x && !lastX)  artifactSystem.triggerManualShot(currentTime);
            if (gamepad2.left_bumper && !lastLB)  artifactSystem.triggerColorShot("G");
            if (gamepad2.right_bumper && !lastRB) artifactSystem.triggerColorShot("P");

            // Manual / Inspection
            boolean ltEdge = (gamepad2.left_trigger > 0.5) && !lastLT;
            boolean rtEdge = (gamepad2.right_trigger > 0.5) && !lastRT;
            boolean xEdge  = gamepad2.x && !lastX;

            if (gamepad2.left_trigger > 0.5)  artifactSystem.inspectSlot("G");
            if (gamepad2.right_trigger > 0.5) artifactSystem.inspectSlot("P");

            // System Update
            artifactSystem.update(currentTime, ltEdge, rtEdge, xEdge);
            sorter.update();
            follower.update();

            // Store history
            lastY = gamepad2.y; lastX = gamepad2.x; lastB = gamepad2.b;
            lastLB = gamepad2.left_bumper; lastRB = gamepad2.right_bumper;
            lastDpadUp = gamepad2.dpad_up; lastDpadDown = gamepad2.dpad_down;
            lastDpadLeft = gamepad2.dpad_left; lastDpadRight = gamepad2.dpad_right;
            lastLT = (gamepad2.left_trigger > 0.5); lastRT = (gamepad2.right_trigger > 0.5);

            telemetry.update();
        }

        artifactSystem.resetSlot();
        intake.stop();
    }
}
