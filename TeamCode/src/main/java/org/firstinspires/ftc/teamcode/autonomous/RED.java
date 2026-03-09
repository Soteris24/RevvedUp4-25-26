package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;

@Autonomous(name = "AutonomousRED")
public class RED extends BaseAutonomous {

    @Override
    protected Pose getStartPose() {
        return new Pose(0, 0, Math.toRadians(45));
    }

    @Override
    protected Pose getShootingPose() {
        return new Pose(-28, -28, Math.toRadians(45));
    }

    @Override
    protected Pose getPos1() {
        return new Pose(-23, -36, Math.toRadians(-1));
    }

    @Override
    protected Pose getPos1Forward() {
        return new Pose(3, -37, Math.toRadians(-3));
    }

    @Override
    protected Pose getPos2() {
        return new Pose(-22, -59, Math.toRadians(-1));
    }

    @Override
    protected Pose getPos2Forward() {
        return new Pose(2, -60, Math.toRadians(-3));
    }
}