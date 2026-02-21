package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter2;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;
@Disabled
@TeleOp(name = "SemiAutomated")
public class SemiAutomated extends LinearOpMode {

    RobotHardware hw;
    Drivetrain drive;
    Shooter2 sh;
    Sorter sorter;
    Intake intake;

    private int targetRpm = 2650;
    private long lastRpmAdjustTime = 0;

    @Override
    public void runOpMode() {
        hw = new RobotHardware();
        hw.init(hardwareMap);
        drive = new Drivetrain(hw, false);
        intake = new Intake(hw, false);
        sh = new Shooter2(hw, false);
        sorter = new Sorter(hw, false);

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
            long currentTime = System.currentTimeMillis();

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (Math.abs(y) < 0.05) y = 0;
            if (Math.abs(x) < 0.05) x = 0;
            if (Math.abs(rx) < 0.05) rx = 0;

            drive.drive(y, x, rx);

            intake.intake(gamepad1.y, gamepad1.b, currentTime);

            if (gamepad2.dpad_up) {
                sh.setTargetVelRPM(targetRpm);
            } else if (gamepad2.dpad_down) {
                sh.setTargetVelRPM(0);
            }

            if (currentTime - lastRpmAdjustTime > 100) {
                if (gamepad2.dpad_left) {
                    targetRpm -= 50;
                    lastRpmAdjustTime = currentTime;
                } else if (gamepad2.dpad_right) {
                    targetRpm += 50;
                    lastRpmAdjustTime = currentTime;
                }
            }

            if (gamepad2.left_bumper) sorter.moveDegrees(120);
            if (gamepad2.right_bumper) sorter.moveDegrees(-120);
            if (gamepad2.left_trigger > 0.1) sorter.moveDegrees(75);
            if (gamepad2.right_trigger > 0.1) sorter.moveDegrees(-75);
            if (gamepad2.x) sorter.transferArtifact(currentTime);

            sleep(10);
        }

        sh.setTargetVelRPM(0);
        intake.stop();
        drive.stop();
        sorter.resetPID();
    }
}
