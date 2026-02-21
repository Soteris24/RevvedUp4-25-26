package org.firstinspires.ftc.teamcode.utils;

public class Convertor {

    public static final double SORTER_MOTOR_TICKS_PER_REV = 8192.0; //28.0 //8192.0
    public static final double SORTER_GEAR_RATIO = 1; // 5.23*2.89

    public static final double SORTER_TICKS_PER_REV = SORTER_MOTOR_TICKS_PER_REV * SORTER_GEAR_RATIO;
    public static final double SORTER_TICKS_PER_DEG = SORTER_TICKS_PER_REV / 360;

    public static final double SHOOTER_MOTOR_TICKS_PER_REV = 28.0;
    public static final double SHOOTER_GEAR_RATIO = 1.0;
    public static final double SHOOTER_TICKS_PER_REV = SHOOTER_MOTOR_TICKS_PER_REV * SHOOTER_GEAR_RATIO;

    public static int toTicks(double deg){
        return (int) (SORTER_TICKS_PER_DEG * deg);
    }

    public static double toDegrees(int ticks) {
        return (ticks / SORTER_TICKS_PER_DEG);
    }

    public static double shooterTPStoRPM (double tps) {
        return (tps / SHOOTER_TICKS_PER_REV) * 60;
    }

    public static double shooterRPMtoTPS(double rpm) {
        return (rpm / 60.0) * SHOOTER_TICKS_PER_REV;
    }

}
