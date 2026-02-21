package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Disabled
@Configurable
@TeleOp( name  = "shooterTest")
public class ShooterTest extends LinearOpMode {

    RobotHardware hw;
    Shooter shooter;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    public static double ShooterTarget = 3500;

    @Override
    public void runOpMode()  {
        hw = new RobotHardware();
        hw.init(hardwareMap);
        shooter = new Shooter(hw, true);

        shooter.setTargetVel(ShooterTarget);
        waitForStart();

        while (opModeIsActive()) {

            shooter.targetVel = ShooterTarget;

            if (gamepad1.dpad_up && !lastDpadUp) {
                shooter.targetVel += 100;
            } else if (gamepad1.dpad_down && !lastDpadDown) {
                shooter.targetVel -= 100;
            }

            lastDpadDown = gamepad1.dpad_down;
            lastDpadUp = gamepad1.dpad_up;

            shooter.setTargetVel(shooter.targetVel);
            shooter.update();


        }
    }
}
