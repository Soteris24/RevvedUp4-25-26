package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;
@Disabled
@TeleOp(name = "colorSensorTest")
public class colorSensorTest extends LinearOpMode {

    RobotHardware hw;
    Sorter sorter;
    //ArtifactSorter artifactSorter;
    PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    @Override
    public void runOpMode()  {

        hw = new RobotHardware();
        hw.init(hardwareMap);

        //sorter = new Sorter(hw, false);
        //artifactSorter = new ArtifactSorter(hw, telemetry,  sorter, true);

        waitForStart();

        while (opModeIsActive()){
            //artifactSorter.detect2(gamepad1.right_bumper, gamepad1.left_bumper);
            telemetry.update();
            panelsTelemetry.getFtcTelemetry().update();
        }
    }
}
