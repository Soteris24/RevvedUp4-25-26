package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class RobotHardware {

    public DcMotor leftFront, leftBack, rightFront, rightBack, intake;
    public DcMotorEx sorter, leftShooter, rightShooter;
    public Servo sorterTransfer;
    public ColorRangeSensor colorSensor, colorSensor2;
    public static double transferIdle = 0.4;
    public static double transferPush = 0.22; //0.25
    public static double DEG_PER_SLOT = 120;
    public int sorterTarget = 0;

    public HardwareMap hardwareMap;
    public ElapsedTime timer = new ElapsedTime();
    public int shotsRemaining = 0;
    String[] storedArtifacts = new String[3];

    public int artifactCount = 0;
    public boolean artifactPresent = false;
    public String[] motif = {"P", "P", "G"};
    public boolean motifSortingEnabled = false;
    int targetSlot;

    PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

    public void init(HardwareMap hwMap) {

        this.hardwareMap = hwMap;

        this.leftFront = hwMap.get(DcMotor.class, "leftFront");
        this.leftBack = hwMap.get(DcMotor.class, "leftBack");
        this.rightFront = hwMap.get(DcMotor.class, "rightFront");
        this.rightBack = hwMap.get(DcMotor.class, "rightBack");

        DcMotor[] drivetrain = {leftFront, leftBack, rightFront, rightBack};

        for (DcMotor dm : drivetrain) {
            dm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            dm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        this.leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        this.leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        this.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        this.intake = hwMap.get(DcMotor.class, "intake");

        this.intake.setDirection(DcMotorSimple.Direction.REVERSE);

        this.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.sorter = hwMap.get(DcMotorEx.class, "sorter");
        this.leftShooter = hwMap.get(DcMotorEx.class, "leftShooter");
        this.rightShooter = hwMap.get(DcMotorEx.class, "rightShooter");
        this.sorterTransfer = hwMap.get(Servo.class, "sorterTransfer");

        this.leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightShooter.setDirection(DcMotorSimple.Direction.FORWARD);

        this.colorSensor = hwMap.get(ColorRangeSensor.class, "colorSensor");
        this.colorSensor2 = hwMap.get(ColorRangeSensor.class, "colorSensor2");

        this.sorter.setDirection(DcMotorSimple.Direction.REVERSE);
        this.sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.sorter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.sorter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor[] shooter = {leftShooter, rightShooter};

        for (DcMotor m : shooter) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        this.timer.reset();

        this.sorterTransfer.setPosition(transferIdle);

    }
    public double getBatteryVoltage() {
        double voltage = 0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            voltage = Math.max(voltage, sensor.getVoltage());
        }
        return voltage;
    }

}


