package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;

@Autonomous(name = "AutonomousREDfar")
public class REDfar extends BaseAutonomous1 {

    @Override
    protected Pose getStartPose() {
        return new Pose(0, 0, Math.toRadians(0));
    }

    @Override
    protected Pose getShootingPose() {
        return new Pose(6, -0.2, Math.toRadians(-28));
    }

    @Override
    protected Pose getPos1() {
        return new Pose(27, -14, Math.toRadians(-90));
    }

    @Override
    protected Pose getPos1Forward() {
        return new Pose(27, -44, Math.toRadians(-90));
    }

    @Override
    protected Pose getPos2() {
        return new Pose(6, -24, Math.toRadians(-90));
    }

    @Override
    protected Pose getPos2Forward() {
        return new Pose(6, -45, Math.toRadians(-90));
    }
}