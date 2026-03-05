package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.Convertor;

import java.util.Objects;
// reverted changes

@Disabled
@TeleOp(name = "sorterTest")
public class Sorterold extends LinearOpMode {

    RobotHardware r;
    public Convertor convertor;

    public PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    public static double sorterP = 0.006;
    public static double sorterI = 0;
    public static double sorterD = 0.001;
    PIDCoefficients sorterCoefficients = new PIDCoefficients(sorterP,sorterI,sorterD);
    PIDController sorterPID = new PIDController(sorterCoefficients);

    public static double sorterReference = 360;

    long lastTime = System.currentTimeMillis();

    @Override
    public void runOpMode()  {
        r.init(hardwareMap);
        r.sorter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (opModeIsActive()) {

            sorterPID.setCoefficients(new PIDCoefficients(sorterP, sorterI, sorterD));

            long currentTime = System.currentTimeMillis();
            double dt = (currentTime - lastTime) / 1000.0;

            if (dt <= 0) dt = 0.001;

            if (gamepad2.dpad_up) {
                r.sorterTarget = 100;
            } else if (gamepad2.dpad_down) {
                r.sorterTarget = 0;
            }

            double error = r.sorterTarget - r.sorter.getCurrentPosition();
            double power = sorterPID.update(error, dt);

            r.sorter.setPower(power);

            r.panelsTelemetry.getTelemetry().addData("Position", r.sorter.getCurrentPosition());
            r.panelsTelemetry.getTelemetry().addData("Target", r.sorterTarget);
            r.panelsTelemetry.getTelemetry().addData("error", error);
            r.panelsTelemetry.getTelemetry().update();
            lastTime = currentTime;
        }

        sorterPID.reset();
        r.sorter.setPower(0);

    }

    public enum FireState {
        IDLE, SPIN, TRANSFER, SHOOT, RETURN, RESET
    }

    FireState fireState = FireState.IDLE;


    public void artifactDetection() {

        int red = r.colorSensor.red();
        int green = r.colorSensor.green();
        int blue = r.colorSensor.blue();
        int alpha = r.colorSensor.alpha();

        boolean artifactDetected = alpha > 1500 && !(blue > red && blue > green);

        if (artifactDetected) {
            this.r.artifactPresent = true;
            String color = "UNKNOWN";

            if (red > blue && red > green) {
                color = "P";
            }
            else if (green > blue && green > red) {
                    color = "G";
                }

            if (!color.equals("UNKNOWN") && r.artifactCount < r.storedArtifacts.length) {
                this.r.storedArtifacts[r.artifactCount] = color;
                r.artifactCount++;
            } else {
                this.r.artifactPresent = false;
            }

            panelsTelemetry.getTelemetry().addData("red", red);
            panelsTelemetry.getTelemetry().addData("blue", blue);
            panelsTelemetry.getTelemetry().addData("green", green);
            panelsTelemetry.getTelemetry().addData("alpha", alpha);
            panelsTelemetry.getTelemetry().addData("artifactCount", this.r.artifactCount);
            panelsTelemetry.getTelemetry().addData("artifactDetected", r.artifactPresent);
            panelsTelemetry.getTelemetry().addData("colors", color);
            panelsTelemetry.getTelemetry().addData("alpha", alpha);

            panelsTelemetry.getTelemetry().update();

        }
    }

    public void fireAll() {
        switch (fireState) {
            case IDLE:
                if (r.artifactCount >= 1) {
                    spinSorter(60);
                    shooterSetPower(1);
                    setFireState(FireState.SPIN);
                }
                else {
                    shooterSetPower(0);
                    setFireState(FireState.IDLE);
                }
                break;
            case SPIN:
                if (sorterAtTarget()) {
                    r.sorterTransfer.setPosition(r.transferPush);
                    r.timer.reset();
                    setFireState(FireState.TRANSFER);
                }
                break;
            case TRANSFER:
                if (r.timer.seconds() >= 0.5) {
                    r.sorterTransfer.setPosition(r.transferIdle);
                    r.timer.reset();
                    setFireState(FireState.SHOOT);
                }
                break;
            case SHOOT:
                if (r.timer.seconds() >= 0.5) {
                    r.artifactCount--;
                    setFireState(FireState.RETURN);
                }
                break;
            case RETURN:
                if (r.artifactCount > 0) {
                    spinSorter(120);
                    setFireState(FireState.SPIN);
                }
                else {
                    spinSorter(60);
                    shooterSetPower(0);
                    setFireState(FireState.RESET);
                }
            case RESET:
                if (sorterAtTarget()) {
                    r.sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    r.sorter.setTargetPosition(0);
                    r.sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setFireState(FireState.IDLE);
                }
        }
    }

    public void motifSorting(){
        if (!r.motifSortingEnabled) return;
        if (r.artifactCount < 3) return;

        for (int i = 0;  i < r.storedArtifacts.length; i++) {
            String targetArtifact = r.motif[i];

            for (int index = 0; index < r.storedArtifacts.length; index++) {
                if (Objects.equals(r.storedArtifacts[index], targetArtifact)) {
                    r.targetSlot = index;
                    break;
                }
            }
        }
    }

    public void spinSorter(double degrees) {
        r.sorterTarget += convertor.toTicks(degrees);
        spinSorter(60);
        r.sorter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moveToSlot(int currentSlot, int targetSlot) {
       int deltaCW = (targetSlot - currentSlot + 3) % 3;
       int deltaCCW = (currentSlot - targetSlot + 3) % 3;

       double degreesToMove = (deltaCW <= deltaCCW ? deltaCW : -deltaCCW) * r.DEG_PER_SLOT;
       degreesToMove += Math.signum(degreesToMove) * 60;

       spinSorter(degreesToMove);
    }

//    public void moveSorter() {
//
//        r.sorterTarget = sorterConvertor.toTicks(sorterReference);
//
//        long lastTime = System.currentTimeMillis();
//
//        while (!sorterAtTarget() && opModeIsActive()) {
//
//            r.sorter.setPower(power);
//
//            panelsTelemetry.getTelemetry().addData("target", r.sorterTarget);
//            panelsTelemetry.getTelemetry().addData("Position", r.sorter.getCurrentPosition());
//            panelsTelemetry.getTelemetry().addData("error", error);
//
//            lastTime = currentTime;
//        }
//        r.sorter.setPower(0);
//        sorterPID.reset();
//    }

    public void sorterUpdate(double dt) {
        double error = r.sorterTarget - r.sorter.getCurrentPosition();
        double power = sorterPID.update(error, dt);

        r.sorter.setPower(power);

        if (Math.abs(error) < 15){
            r.sorter.setPower(0);
        }

        r.panelsTelemetry.getTelemetry().addData("Position", r.sorter.getCurrentPosition());
        r.panelsTelemetry.getTelemetry().addData("Target", r.sorterTarget);
        r.panelsTelemetry.getTelemetry().addData("error", error);
        r.panelsTelemetry.getTelemetry().update();
    }

    public void shooterSetPower(double power) {
        r.leftShooter.setPower(power);
        r.rightShooter.setPower(power);
    }

    public boolean sorterAtTarget(){
        return Math.abs(r.sorter.getCurrentPosition() - r.sorter.getTargetPosition()) < 15;
    }

    public void setFireState(FireState state) {
        fireState = state;
    }

}
