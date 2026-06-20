package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;

@Autonomous(name = "AutonomousBLUEfar")
public class BLUEfar extends BaseAutonomous1 {


    @Override
    protected Pose getStartPose() {
        return new Pose(0, 0, Math.toRadians(0));
    }

    @Override
    protected Pose getShootingPose() {
        return new Pose(4, 1, Math.toRadians(26));
    }

    @Override
    protected Pose getPos1() {
        return new Pose(28, 10, Math.toRadians(90));
    }

    @Override
    protected Pose getPos1Forward() {
        return new Pose(28, 50, Math.toRadians(90));
    }

    @Override
    protected Pose getPos2() {
        return new Pose(6, 35, Math.toRadians(90));
    }

    @Override
    protected Pose getPos2Forward() {
        return new Pose(6, 55, Math.toRadians(90));
    }

    @Override
    protected Pose getHumanPlayer() {
        return new Pose(8, 50, Math.toRadians(90));
    }

    @Override
    protected Pose getHumanPlayer2() {
        return new Pose(35, 65, Math.toRadians(15));
    }

    @Override
    protected int getPipeline() {return 9;}
}