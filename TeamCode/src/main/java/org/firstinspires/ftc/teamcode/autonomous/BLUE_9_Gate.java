package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BLUE - 9 Ball + Gate", group = "Blue Close")
public class BLUE_9_Gate extends BaseAutonomous {

    @Override
    protected Mode getMode() {
        return Mode.NINE_WITH_GATE;
    }

    @Override protected Pose getStartPose()    { return new Pose(0, 0, Math.toRadians(47)); }
    @Override protected Pose getShootingPose() { return new Pose(-35, -35, Math.toRadians(45)); }
    @Override protected Pose getMotifPose()    { return new Pose(-30, -30, Math.toRadians(-45)); }
    @Override protected Pose getPos1()         { return new Pose(-40, -25, Math.toRadians(90)); }
    @Override protected Pose getPos1Forward()  { return new Pose(-40, 7, Math.toRadians(90)); }
    @Override protected Pose getPos2()         { return new Pose(-65, -25, Math.toRadians(90)); }
    @Override protected Pose getPos2Forward()  { return new Pose(-65, 20, Math.toRadians(75)); }
    @Override protected Pose getPos3()         { return new Pose(-95, -30, Math.toRadians(90)); }
    @Override protected Pose getPos3Forward()  { return new Pose(-95, 15, Math.toRadians(90)); }
    @Override protected Pose getPosgate()      { return new Pose(-50, 12, Math.toRadians(90)); }
    @Override protected Pose getPosgateready() { return new Pose(-50, -5, Math.toRadians(90)); }
    @Override protected int getPipeline() {return 9;}
}
