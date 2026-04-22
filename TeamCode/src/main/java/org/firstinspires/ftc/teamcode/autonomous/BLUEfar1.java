package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;

@Autonomous(name = "AutonomousBLUEfar1")
public class BLUEfar1 extends BaseAutonomous2 {


    @Override
    protected Pose getStartPose() {
        return new Pose(0, 0, Math.toRadians(0));
    }

    @Override
    protected Pose getShootingPose() {
        return new Pose(4, 0.2, Math.toRadians(28));
    }

    @Override
    protected Pose getPos1() {
        return new Pose(26, 10, Math.toRadians(90));
    }

    @Override
    protected Pose getPos1Forward() {
        return new Pose(26, 35, Math.toRadians(90));
    }

    @Override
    protected Pose getPos2() {
        return new Pose(6, 35, Math.toRadians(90));
    }

    @Override
    protected Pose getPos2Forward() {
        return new Pose(6, 55, Math.toRadians(90));
    }
}