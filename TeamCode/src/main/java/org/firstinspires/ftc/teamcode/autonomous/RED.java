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
        return new Pose(-25, -25, Math.toRadians(45));
    }

    @Override
    protected Pose getPos1() {
        return new Pose(-15, -45, Math.toRadians(0));
    }

    @Override
    protected Pose getPos1Forward() {
        return new Pose(10, -45, Math.toRadians(0));
    }

    @Override
    protected Pose getPos2() {
        return new Pose(-15, -66, Math.toRadians(0));
    }

    @Override
    protected Pose getPosgate() {
        return new Pose(15, -55, Math.toRadians(0));
    }

    @Override
    protected Pose getPosgateready() {
        return new Pose(0, -55, Math.toRadians(0));
    }

    @Override
    protected Pose getPos2Forward() {
        return new Pose(10, -66, Math.toRadians(0));
    }
}