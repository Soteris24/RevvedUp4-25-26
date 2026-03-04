package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;

@Autonomous(name = "AutonomousBLUEfar")
public class BLUEfar extends BaseAutonomous {

    @Override
    protected Pose getStartPose() {
        return new Pose(0, 0, Math.toRadians(0));
    }

    @Override
    protected Pose getShootingPose() {
        return new Pose(4, 0.2, Math.toRadians(21));
    }

    @Override
    protected Pose getPos1() {
        return new Pose(28, 18, Math.toRadians(90));
    }

    @Override
    protected Pose getPos1Forward() {
        return new Pose(26, 46, Math.toRadians(90));
    }

    @Override
    protected Pose getPos2() {
        return new Pose(6, 24, Math.toRadians(90));
    }

    @Override
    protected Pose getPos2Forward() {
        return new Pose(6, 45, Math.toRadians(90));
    }
}