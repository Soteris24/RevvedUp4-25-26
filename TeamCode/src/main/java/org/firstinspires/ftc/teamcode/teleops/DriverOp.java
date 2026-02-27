package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.mathUtils.ShooterCalculator;
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

    @Override
    public void runOpMode() {
        hw = new RobotHardware();
        hw.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);

        drivetrain = new Drivetrain(hw, follower, true);
        intake         = new Intake(hw, false);
        sorter         = new Sorter(hw, false);
        shooter        = new Shooter2(hw, false);
        artifactSystem = new ArtifactSystem(hw, telemetry, sorter, shooter, intake, false);
        calc           = new ShooterCalculator(distanceTable, rpmTable);

        waitForStart();

        while (opModeIsActive()) {
            double currentTime = getRuntime();

            shooter.setPIDFCoefficients();

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

            // --- ArtifactSystem (gamepad2) ---
            // Button map:
            //   dpad_up    — enter SHOOT (long RPM) from INTAKE / SET long  RPM in SHOOT
            //   dpad_down  — SET short RPM in SHOOT
            //   dpad_left  — enter SHOOT (short RPM) from INTAKE
            //   dpad_right — exit SHOOT / MANUAL → INTAKE
            //   left_bumper  INTAKE: manual GREEN   | SHOOT: fire GREEN
            //   right_bumper INTAKE: manual PURPLE  | SHOOT: fire PURPLE
            //   left_trigger  INTAKE: inspect GREEN  | MANUAL: rotate left
            //   right_trigger INTAKE: inspect PURPLE | MANUAL: rotate right
            //   y — toggle intake (INTAKE) / fire current slot (MANUAL)
            //   x — fire current slot in SHOOT state (no rotation)
            //   a — reverse intake (hold)
            artifactSystem.update(
                    gamepad2.dpad_up,
                    gamepad2.dpad_down,
                    gamepad2.dpad_left,
                    gamepad2.dpad_right,
                    gamepad2.left_trigger  > 0.5,
                    gamepad2.right_trigger > 0.5,
                    gamepad2.left_bumper,
                    gamepad2.right_bumper,
                    gamepad2.y,           // intake toggle (INTAKE) / fire current slot (MANUAL)
                    gamepad2.x,           // fire current slot (SHOOT)
                    gamepad2.a,           // reverse intake
                    gamepad2.b,           // auto fire all
                    currentTime
            );
            if (gamepad1.a) {
                drivetrain.drive(y, x, rotInput);
            }
            follower.setMaxPower(1);
            sorter.update();
            follower.update();
            drivetrain.telemetryOn = true;
        }

        drivetrain.stop();
        sorter.resetPID();
        artifactSystem.resetSlot();
        intake.stop();
    }
}