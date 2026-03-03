package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled
@Autonomous(name = "PoseCalibration")
public class calibrate extends LinearOpMode {

    private Follower follower;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(45));

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        telemetry.addData("Status", "Ready - push start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            Pose pose = follower.getPose();
            telemetry.addData("X", "%.3f", pose.getX());
            telemetry.addData("Y", "%.3f", pose.getY());
            telemetry.addData("Heading (deg)", "%.3f", Math.toDegrees(pose.getHeading()));
            telemetry.addData("Heading (rad)", "%.4f", pose.getHeading());
            telemetry.update();

            sleep(10);
        }
    }
}