package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;
@Disabled
@TeleOp(name  = "sorterTest")
public class SorterTest extends LinearOpMode {

    RobotHardware hw;
    Sorter sorter;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    PanelsTelemetry tel = PanelsTelemetry.INSTANCE;

    @Override
    public void runOpMode()  {
        hw = new RobotHardware();
        hw.init(hardwareMap);
        sorter = new Sorter(hw, true);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up && !lastDpadUp) {
                sorter.moveDegrees(120);
            } else if (gamepad1.dpad_down && !lastDpadDown) {
                sorter.moveDegrees(-120);
            }

            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;

            sorter.update();


            telemetry.addData("current Position", hw.sorter.getCurrentPosition());
            telemetry.addData("Target Position", sorter.targetTicks);
            telemetry.update();

            tel.getTelemetry().addData("current Position", hw.sorter.getCurrentPosition());
            tel.getTelemetry().addData("target Position", sorter.targetTicks);
            tel.getTelemetry().update();
        }
    }
}
