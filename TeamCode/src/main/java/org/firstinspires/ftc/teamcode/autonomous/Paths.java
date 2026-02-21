package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

public class Paths {

    public final PathChain path1;
    public final PathChain path2;
    public final PathChain path3;

    public Paths(Follower follower) {

        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(122.908, 124.731, Math.toRadians(45)),
                        new Pose(90.879, 94.004, Math.toRadians(45))
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(45),
                        Math.toRadians(45)
                )
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(90.879, 94.004, Math.toRadians(45)),
                        new Pose(95.356, 83.769, Math.toRadians(45))
                ))
                .setConstantHeadingInterpolation(Math.toRadians(45))
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(95.356, 83.769),
                        new Pose(127.477, 83.590)
                ))
                .setTangentHeadingInterpolation()
                .build();
    }
}
