package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Shooter2;
import org.firstinspires.ftc.teamcode.subsystems.Webcam;

@Configurable
@TeleOp(name = "ShootingTest")

public class ShootingTest extends LinearOpMode {
    RobotHardware hw;
    Shooter2 shooter;
    Webcam webcam;

    PanelsTelemetry tel = PanelsTelemetry.INSTANCE;

    double alpha = 0.1; // smoothing factor
    double avgHz = 0;

    long lastTime = System.nanoTime();

    public static double target = 0;
    @Override
    public void runOpMode()  {
        hw = new RobotHardware();
        hw.init(hardwareMap);
        shooter = new Shooter2(hw, true);
        webcam = new Webcam();
        webcam.initTagProcessor(hardwareMap, telemetry, true);
        waitForStart();

        while (opModeIsActive()) {

            webcam.update();
            double[] distOffset = webcam.getDistanceAndOffset();
            double distance = distOffset[0];
            double rotationOffset = distOffset[1];

            shooter.setPIDFCoefficients();
            shooter.setTargetVelRPM(target);
            shooter.updateTelemetry();

            long now = System.nanoTime();
            double dt = (now - lastTime) / 1_000_000_000.0;
            lastTime = now;

            double instantHz = 1.0 / dt;
            avgHz = avgHz * (1 - alpha) + instantHz * alpha;

            telemetry.addData("Loop Hz (smoothed)", avgHz);

            telemetry.addData("Distance to Goal", distance >= 0 ? distance : "N/A");
            telemetry.addData("Rotation Offset", distance >= 0 ? rotationOffset : "N/A");
            telemetry.addData("distance", distance);
            tel.getFtcTelemetry().addData("distance", distance);
            tel.getFtcTelemetry().update();
            telemetry.update();
        }
    }
}
