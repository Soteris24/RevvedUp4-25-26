package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.mathUtils.ShooterCalculator;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArtifactSystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter2;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;
import org.firstinspires.ftc.teamcode.utils.LatchedEdgeButton;

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

    double[] distanceTable = {24, 48, 72, 144};
    double[] rpmTable      = {1650, 1850, 2050, 2650};

    PanelsTelemetry tel = PanelsTelemetry.INSTANCE;
    ElapsedTime loopTimer = new ElapsedTime();
    List<LynxModule> allHubs;

    public static double edgeLatchSeconds = 0.12;

    LatchedEdgeButton yButton = new LatchedEdgeButton(edgeLatchSeconds);
    LatchedEdgeButton xButton = new LatchedEdgeButton(edgeLatchSeconds);
    LatchedEdgeButton bButton = new LatchedEdgeButton(edgeLatchSeconds);
    LatchedEdgeButton lbButton = new LatchedEdgeButton(edgeLatchSeconds);
    LatchedEdgeButton rbButton = new LatchedEdgeButton(edgeLatchSeconds);
    LatchedEdgeButton ltButton = new LatchedEdgeButton(edgeLatchSeconds);
    LatchedEdgeButton rtButton = new LatchedEdgeButton(edgeLatchSeconds);
    LatchedEdgeButton dpadUpButton = new LatchedEdgeButton(edgeLatchSeconds);
    LatchedEdgeButton dpadDownButton = new LatchedEdgeButton(edgeLatchSeconds);
    LatchedEdgeButton dpadLeftButton = new LatchedEdgeButton(edgeLatchSeconds);
    LatchedEdgeButton dpadRightButton = new LatchedEdgeButton(edgeLatchSeconds);

    String lastEdgeSeen = "none";
    String lastEdgeResult = "none";
    int seenEdgeCount = 0;
    int acceptedEdgeCount = 0;
    int rejectedEdgeCount = 0;

    @Override
    public void runOpMode() {
        hw = new RobotHardware();
        hw.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);

        drivetrain = new Drivetrain(hw, follower, false);
        intake         = new Intake(hw, false);
        sorter         = new Sorter(hw, intake, false);
        shooter        = new Shooter2(hw, false);
        artifactSystem = new ArtifactSystem(hw, telemetry, sorter, shooter, intake, false);
        calc           = new ShooterCalculator(distanceTable, rpmTable);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {
            double currentTime = getRuntime();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            shooter.setPIDFCoefficients();
            telemetry.addData("Loop ms", loopTimer.milliseconds());
            loopTimer.reset();

            // --- Drivetrain (gamepad1) ---
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            double rotInput = rx;

            if (gamepad1.a) {
                rotInput = drivetrain.facePoint(25+15, 0); // returns a rotation power
            }
            if (gamepad1.b) {
                rotInput = drivetrain.facePoint(0, 0); // returns a rotation power
            }

            drivetrain.drive(y, x, rotInput);


            Pose currentPose = follower.getPose();
            double dx = 0 - currentPose.getX(); // goal coordinates
            double dy = 0 - currentPose.getY();
            artifactSystem.currentDistance = Math.hypot(dx, dy);

            refreshButtonLatches(currentTime);

            boolean upEdge = dpadUpButton.consume(currentTime);
            boolean downEdge = dpadDownButton.consume(currentTime);
            boolean leftEdge = dpadLeftButton.consume(currentTime);
            boolean rightEdge = dpadRightButton.consume(currentTime);
            boolean yEdge = yButton.consume(currentTime);
            boolean xEdge = xButton.consume(currentTime);
            boolean bEdge = bButton.consume(currentTime);
            boolean lbEdge = lbButton.consume(currentTime);
            boolean rbEdge = rbButton.consume(currentTime);
            boolean ltEdge = ltButton.consume(currentTime);
            boolean rtEdge = rtButton.consume(currentTime);

            if (rightEdge) {
                recordAction("dpad_right", artifactSystem.switchToIntake());
            }
            if (leftEdge) {
                recordAction("dpad_left", artifactSystem.switchToShooting(ArtifactSystem.SHORT_RPM, true));
            }
            if (upEdge) {
                recordAction("dpad_up", artifactSystem.switchToShooting(ArtifactSystem.LONG_RPM, false));
            }
            if (downEdge) {
                recordAction("dpad_down", artifactSystem.switchToShooting(ArtifactSystem.SHORT_RPM, false));
            }

            if (yEdge) {
                recordAction("y", artifactSystem.toggleIntake());
            }

            if (artifactSystem.robotState == ArtifactSystem.RobotState.INTAKE) {
                artifactSystem.setIntakeReverse(gamepad2.a, currentTime);
                if (lbEdge) {
                    recordAction("left_bumper", artifactSystem.manualDetect("G"));
                }
                if (rbEdge) {
                    recordAction("right_bumper", artifactSystem.manualDetect("P"));
                }
                if (ltEdge) {
                    recordAction("left_trigger", artifactSystem.inspectSlot("G"));
                }
                if (rtEdge) {
                    recordAction("right_trigger", artifactSystem.inspectSlot("P"));
                }
            } else if (artifactSystem.robotState == ArtifactSystem.RobotState.SHOOTING) {
                if (bEdge) {
                    recordAction("b", artifactSystem.triggerAutoFire());
                }
                if (xEdge) {
                    recordAction("x", artifactSystem.triggerManualShot(currentTime));
                }
                if (lbEdge) {
                    recordAction("left_bumper", artifactSystem.triggerColorShot("G"));
                }
                if (rbEdge) {
                    recordAction("right_bumper", artifactSystem.triggerColorShot("P"));
                }
            }

            artifactSystem.update(currentTime, ltEdge, rtEdge, false);
            if (gamepad1.a) {
                drivetrain.drive(y, x, rotInput);
            }
            follower.setMaxPower(1);
            sorter.update();
            follower.update();
            drivetrain.telemetryOn = false;

            telemetry.addData("Input Edge Seen", lastEdgeSeen);
            telemetry.addData("Input Edge Result", lastEdgeResult);
            telemetry.addData("Artifact Action", artifactSystem.getLastActionMessage());
            telemetry.addData("Edge Counts", "seen=%d accepted=%d rejected=%d", seenEdgeCount, acceptedEdgeCount, rejectedEdgeCount);
            telemetry.addData(
                    "Latched Pending",
                    "Y=%s X=%s B=%s LB=%s RB=%s LT=%s RT=%s U=%s D=%s L=%s R=%s",
                    yButton.isPending(currentTime),
                    xButton.isPending(currentTime),
                    bButton.isPending(currentTime),
                    lbButton.isPending(currentTime),
                    rbButton.isPending(currentTime),
                    ltButton.isPending(currentTime),
                    rtButton.isPending(currentTime),
                    dpadUpButton.isPending(currentTime),
                    dpadDownButton.isPending(currentTime),
                    dpadLeftButton.isPending(currentTime),
                    dpadRightButton.isPending(currentTime)
            );

            telemetry.update();
        }

        drivetrain.stop();
        sorter.resetPID();
        artifactSystem.resetSlot();
        intake.stop();
    }

    private void refreshButtonLatches(double currentTime) {
        dpadUpButton.update(gamepad2.dpad_up, currentTime);
        dpadDownButton.update(gamepad2.dpad_down, currentTime);
        dpadLeftButton.update(gamepad2.dpad_left, currentTime);
        dpadRightButton.update(gamepad2.dpad_right, currentTime);
        yButton.update(gamepad2.y, currentTime);
        xButton.update(gamepad2.x, currentTime);
        bButton.update(gamepad2.b, currentTime);
        lbButton.update(gamepad2.left_bumper, currentTime);
        rbButton.update(gamepad2.right_bumper, currentTime);
        ltButton.update(gamepad2.left_trigger > 0.5, currentTime);
        rtButton.update(gamepad2.right_trigger > 0.5, currentTime);
    }

    private void recordAction(String inputName, boolean accepted) {
        seenEdgeCount++;
        lastEdgeSeen = inputName;
        if (accepted) {
            acceptedEdgeCount++;
        } else {
            rejectedEdgeCount++;
        }
        lastEdgeResult = inputName + " -> " + artifactSystem.getLastActionMessage();
    }
}
