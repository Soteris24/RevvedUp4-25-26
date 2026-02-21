package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Shooter2;

import java.util.StringJoiner;

@Configurable
@TeleOp(name = "ManualOp")
public class ManualOp extends LinearOpMode {
    RobotHardware robotHardware;

    Sorter sorter1;
    Shooter2 shooter;
    Intake intake;
    public static int ShooterFarRPM = 3000;
    public static int ShooterCloseRPM = 2000;

    private boolean lastY = false;
    private boolean intakeOn = false;
    private boolean lastButton = false;
    private boolean reversing = false;
    public  double reverseTime = 0;

    private int transferState = 0;

    @Override
    public void runOpMode()  {
        this.sorter1 = new Sorter();
        this.robotHardware = new RobotHardware();
        this.robotHardware.init(hardwareMap);
        this.shooter = new Shooter2(robotHardware, false);
        this.intake = new Intake(robotHardware, false);

        shooter.setTargetVelRPM(0);

        waitForStart();

        while (opModeIsActive()) {
            this.controlDrivetrain();
            this.intake.intake(gamepad2.y, gamepad2.a, System.currentTimeMillis());
            this.controlSorter();
            this.controlTransfer();
            this.controlShooter();
            this.updateColorDetection();

        }
    }

    void controlDrivetrain() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        robotHardware.leftFront.setPower((y + x + rx));
        robotHardware.leftBack.setPower((y - x + rx));
        robotHardware.rightFront.setPower((y - x - rx));
        robotHardware.rightBack.setPower((y + x - rx));
    }

    void controlShooter() {

            if (gamepad2.dpad_up) {
                shooter.setTargetVelRPM(ShooterFarRPM);
            } else if(gamepad2.dpad_left) {
                shooter.setTargetVelRPM(ShooterCloseRPM);
            } else if (gamepad2.dpad_down) {
                shooter.setTargetVelRPM(0);
            }
    }


        public void intake(boolean button, boolean reverseButton, double currentTime){

            if (button && !lastButton) {
                intakeOn = !intakeOn;
            }

            if (reverseButton) {
                reversing = true;
                reverseTime = currentTime + 0.5;
            }
            if (reversing) {
                robotHardware.intake.setPower(-1);
                if (!reverseButton && currentTime >= reverseTime) {
                    reversing = false;
                }
            } else {
                robotHardware.intake.setPower(intakeOn ? 1 : 0);
            }

            if (intakeOn) {
                shooter.setTargetVelRPM(100);
            }

            lastButton = button;
        }

    void controlTransfer(){
        switch (transferState) {
            case 0:
                if (gamepad2.x) {
                robotHardware.sorterTransfer.setPosition(RobotHardware.transferPush);
                robotHardware.timer.reset();
                transferState = 1;
                }
                break;
            case 1:
                if (robotHardware.timer.seconds() >= 0.3) {
                    robotHardware.sorterTransfer.setPosition(RobotHardware.transferIdle);
                    robotHardware.timer.reset();
                    transferState = 0;
                }
                break;
        }
    }

    void controlSorter(){
            this.robotHardware.sorter.setPower((gamepad2.left_trigger - gamepad2.right_trigger)/ 3.5);
    }

    void updateColorDetection() {

        int red = robotHardware.colorSensor.red();
        int green = robotHardware.colorSensor.green();
        int blue = robotHardware.colorSensor.blue();
        int alpha = robotHardware.colorSensor.alpha();

        boolean artifactDetected = alpha > 150 && !(blue > red && blue > green);

        robotHardware.panelsTelemetry.getTelemetry().addData("red", red);
        robotHardware.panelsTelemetry.getTelemetry().addData("blue", blue);
        robotHardware.panelsTelemetry.getTelemetry().addData("green", green);
        robotHardware.panelsTelemetry.getTelemetry().addData("alpha", alpha);
        robotHardware.panelsTelemetry.getTelemetry().addData("artifactDetected", artifactDetected);

        robotHardware.panelsTelemetry.getTelemetry().update();
    }

}
