package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;

@Autonomous(name = "AutonomousRED")
public class RED extends BaseAutonomous {

    @Override
    protected Pose getStartPose() {
        return new Pose(0, 0, Math.toRadians(43));
    }

    @Override
    protected Pose getShootingPose() {
        return new Pose(-30, -30, Math.toRadians(45));
    }

    @Override
    protected Pose getPos1() {
        return new Pose(-23, -40, Math.toRadians(0));
    }

    @Override
    protected Pose getPos1Forward() {
        return new Pose(3, -40, Math.toRadians(0));
    }

    @Override
    protected Pose getPos2() {
        return new Pose(-22, -60, Math.toRadians(0));
    }

    @Override
    protected Pose getPosgate() {
        return new Pose(-22, -59, Math.toRadians(0));
    }

    @Override
    protected Pose getPosgateready() {
        return new Pose(-22, -59, Math.toRadians(0));
    }

    @Override
    protected Pose getPos2Forward() {
        return new Pose(2, -60, Math.toRadians(0));
    }
}