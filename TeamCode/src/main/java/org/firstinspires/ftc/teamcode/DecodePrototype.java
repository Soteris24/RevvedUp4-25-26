package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

//revert changes

@Disabled
@TeleOp(name = "RevvedUp3")
public class DecodePrototype extends LinearOpMode {

    RobotHardware robot;
    Sorterold sort;

    private boolean lastY = false;
    private boolean intakeOn = false;

    int shootState = 0;

    @Override
    public void runOpMode() {
        this.robot = new RobotHardware();

        waitForStart();

        this.drive();
        this.intakeControl();
        this.updateTelemetry();

    }



    public void drive(){

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double angle = Math.atan2(y,x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(angle + Math.PI/4);
        double cos = -Math.cos(angle + Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));
        double Max = Math.max (max, rx);

        robot.leftFront.setPower(((power * sin / max) + rx) / Max);
        robot.leftBack.setPower(((power * cos / max) + rx) / Max);
        robot.rightFront.setPower(((power * cos / max) - rx) / Max);
        robot.rightBack.setPower(((power * sin / max) - rx) / Max);

    }

    public void shootAll () {

        switch (shootState) {
            case 0: if (gamepad2.dpad_up) {
                robot.shotsRemaining = 3;
                sort.spinSorter(60);
                sort.shooterSetPower(1);
                shootState = 1;
            }
                break;
            case 1: if (sort.sorterAtTarget()) {
                robot.sorterTransfer.setPosition(robot.transferPush);
                robot.timer.reset();
                shootState = 2;
            }
                break;
            case 2: if (robot.timer.seconds() >= 0.5) {
                robot.sorterTransfer.setPosition(robot.transferIdle);
                robot.timer.reset();
                shootState = 3;
            }
                break;
            case 3: if (robot.timer.seconds() >= 0.5){
                robot.shotsRemaining--;
                shootState = 4;
            }
                break;
            case 4: if (robot.shotsRemaining > 0) {
                sort.spinSorter(120);
                shootState = 1;
            } else {
                sort.shooterSetPower(0);
                sort.spinSorter(60);
                shootState = 5;
            }
                break;
            case 5: if (sort.sorterAtTarget()) {
                robot.sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.sorterTarget = 0;
                robot.sorter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shootState = 0;
            }

        }
    }

    public void intakeControl() {
        boolean currentY = gamepad2.y;

        if (currentY && !lastY) {
            intakeOn = !intakeOn;
        }

        robot.intake.setPower(intakeOn ? 1 : 0);

        lastY = currentY;
    }

    public void updateTelemetry() {
        telemetry.addData("sorterPosition", robot.sorter.getCurrentPosition());
        telemetry.addData("sorterTarget", robot.sorter.getTargetPosition());
        telemetry.addData("shootSate", shootState);
        telemetry.addData("shotsRemaining", robot.shotsRemaining);

        telemetry.update();
    }

}
