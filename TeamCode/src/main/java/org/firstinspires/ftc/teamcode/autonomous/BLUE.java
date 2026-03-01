package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;

@Autonomous(name = "BLUE")
public class BLUE extends BaseAutonomous {

    @Override
    protected Pose getStartPose() {
        return new Pose(0, 0, Math.toRadians(45));
    }

    @Override
    protected Pose getShootingPose() {
        return new Pose(-25, -25, Math.toRadians(45));
    }

    @Override
    protected Pose getPos1() {
        return new Pose(-38, -23, Math.toRadians(90));
    }

    @Override
    protected Pose getPos1Forward() {
        return new Pose(-38, 0, Math.toRadians(90));
    }

    @Override
    protected Pose getPos2() {
        return new Pose(-63, -20, Math.toRadians(90));
    }

    @Override
    protected Pose getPos2Forward() {
        return new Pose(-63, 15, Math.toRadians(90));
    }
}