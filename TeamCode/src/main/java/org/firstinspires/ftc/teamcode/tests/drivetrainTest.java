package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotHardware;
@Disabled
@TeleOp(name = "DT_TEST")
public class drivetrainTest extends LinearOpMode {

    RobotHardware hw;
    ;
    @Override
    public void runOpMode()  {

        hw = new RobotHardware();
        hw.init(hardwareMap);
        hw.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hw.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hw.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hw.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("leftEncoder", hw.leftFront.getCurrentPosition());
            telemetry.addData("StrafeEncoder", hw.rightFront.getCurrentPosition());
            telemetry.addData("rightEncoder", -hw.intake.getCurrentPosition());
            telemetry.update();
        }
    }
}
