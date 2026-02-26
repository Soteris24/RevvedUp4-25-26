package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Webcam;
import org.firstinspires.ftc.teamcode.mathUtils.ShooterCalculator;

@TeleOp(name = " webcamTest")
public class CameraTest extends LinearOpMode {
    Webcam webcam;
    Follower follower;
    ShooterCalculator shooterCalc;
    Drivetrain drivetrain;
    RobotHardware hw;

    String[] motif = {"N/A", "N/A", "N/A"};

    long lastTime = 0;

    @Override
    public void runOpMode() {
        webcam = new Webcam();
        webcam.initTagProcessor(hardwareMap, telemetry, true);

        double[] distanceTable = {24, 48, 72, 144};
        double[] rpmTable = {1650, 1850, 2050, 2650};
        hw = new RobotHardware();
        hw.init(hardwareMap);
        shooterCalc = new ShooterCalculator(distanceTable, rpmTable);
        drivetrain = new Drivetrain(hw, follower, true);

        waitForStart();

        while (opModeIsActive()) {

            long currentTime = System.currentTimeMillis();
            double dt = Math.max((currentTime - lastTime) / 1000.0, 0.001);

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double turnPower = gamepad1.x ? webcam.getAlignCorrection(dt) : rx;

            drivetrain.drive(y, x, turnPower);

            webcam.update();
            double[] distOffset = webcam.getDistanceAndOffset();
            double distance = distOffset[0];
            double rotationOffset = distOffset[1];
            double targetRpm = shooterCalc.calculateRPM(distance);

            if (gamepad1.a) {
                motif = webcam.getMotif();
            }


            telemetry.addData("Target Motif", motif != null ? motif : "UNKNOWN");
            telemetry.addData("Distance to Goal", distance >= 0 ? distance : "N/A");
            telemetry.addData("Calculated RPM", targetRpm > 0 ? targetRpm : "N/A");
            telemetry.addData("Rotation Offset", distance >= 0 ? rotationOffset : "N/A");

            telemetry.update();


            lastTime = currentTime;
        }
        webcam.stop();
    }
}
