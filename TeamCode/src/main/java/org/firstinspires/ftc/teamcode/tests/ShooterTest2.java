package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name = "ShooterTest2")
public class ShooterTest2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx leftShooter  = hardwareMap.get(DcMotorEx.class, "leftShooter");
        DcMotorEx rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");

        // Optional: reverse one
        leftShooter.setDirection(DcMotorEx.Direction.REVERSE);

        // Example PIDF values (tune later)
        leftShooter.setVelocityPIDFCoefficients(0.5, 0.5, 0, 14);
        rightShooter.setVelocityPIDFCoefficients(0.5, 0.5, 0, 14);

        double target = 0; // ticks per second

        waitForStart();

        while (opModeIsActive()) {

            // A → turn on shooter
            if (gamepad1.a) {
                target = 500;  // set your desired TPS here
            }

            // B → stop shooter
            if (gamepad1.b) {
                target = 1000;
            }

            if (gamepad1.y) {
                target = 1500;
            }

            leftShooter.setVelocity(target);
            rightShooter.setVelocity(target);

            // Telemetry
            telemetry.addData("Target TPS", target);
            telemetry.addData("Left Vel", leftShooter.getVelocity());
            telemetry.addData("Right Vel", rightShooter.getVelocity());
            telemetry.update();
        }
    }
}
