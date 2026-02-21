package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.mathUtils.ShooterCalculator;
import org.firstinspires.ftc.teamcode.subsystems.ArtifactSystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter2;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;

@Configurable
@TeleOp(name = "DriverOp")
public class DriverOp extends LinearOpMode {

    RobotHardware     hw;
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

        drivetrain     = new Drivetrain(hw, false);
        intake         = new Intake(hw, false);
        sorter         = new Sorter(hw, false);
        shooter        = new Shooter2(hw, false);
        artifactSystem = new ArtifactSystem(hw, telemetry, sorter, shooter, intake, false);
        calc           = new ShooterCalculator(distanceTable, rpmTable);

        waitForStart();

        while (opModeIsActive()) {
            double currentTime = getRuntime();

            shooter.setPIDFCoefficients();

            // --- Drivetrain ---
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            // Slow rotation while the sorter is physically moving during a shot
            double rotInput = artifactSystem.isActivelyShooting() ? rx / 3.0 : rx;
            drivetrain.drive(y, x, rotInput);

            // --- ArtifactSystem ---
            // gamepad2 map:
            //   dpad_up    — enter SHOOT (long) from intake / bump RPM up in shoot
            //   dpad_down  — bump RPM down in shoot
            //   dpad_left  — enter SHOOT (short) from intake
            //   dpad_right — exit SHOOT / MANUAL → INTAKE
            //   left_bumper  INTAKE: manual GREEN detect   |  SHOOT: fire GREEN
            //   right_bumper INTAKE: manual PURPLE detect  |  SHOOT: fire PURPLE
            //   left_trigger  INTAKE: cycle to next GREEN slot  | MANUAL: rotate left
            //   right_trigger INTAKE: cycle to next PURPLE slot | MANUAL: rotate right
            //   y — toggle intake on/off
            artifactSystem.update(
                    gamepad2.dpad_up,
                    gamepad2.dpad_down,
                    gamepad2.dpad_left,
                    gamepad2.dpad_right,
                    gamepad2.left_trigger  > 0.5,
                    gamepad2.right_trigger > 0.5,
                    gamepad2.left_bumper,
                    gamepad2.right_bumper,
                    gamepad2.y,
                    gamepad2.y,
                    gamepad2.x,
                    currentTime
            );

            // Sorter PID must be called every loop
            sorter.update();
        }

        // Cleanup
        drivetrain.stop();
        sorter.resetPID();
        artifactSystem.resetSlot();
        intake.stop();
    }
}