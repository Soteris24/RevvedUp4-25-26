package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro-Autonomous")
@Configurable
public class testforaxi extends OpMode {
    DcMotor  TopRightMotor, BottomLeftMotor, BottomRightMotor, AssistantShooter, TopLeftMotor;
    DcMotor leftOdo, rightOdo, mid0do; //encoders
    DcMotorEx ShooterMotor,  Collector;    //Ex motor for RPM and velocity measurements

    /* ================= TELEMETRY ================= */
    private TelemetryManager panelsTelemetry;

    /* ================= PATHING =================== */
    private Follower follower;
    private Paths paths;

    /* ================= STATE MACHINE ============= */
    private int pathState = 0;

    /* ================= TIMERS ==================== */
    private Timer pathTimer;
    private Timer opmodeTimer;

    /* ================= START POSE ================ */
    private final Pose startPose = new Pose(72, 8, Math.toRadians(90));

    /* ================= INIT ====================== */
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        TopLeftMotor = hardwareMap.get(DcMotor.class, "TopLeftMotor");
        BottomLeftMotor = hardwareMap.get(DcMotor.class, "BottomLeftMotor");
        TopRightMotor = hardwareMap.get(DcMotor.class, "TopRightMotor");
        BottomRightMotor = hardwareMap.get(DcMotor.class, "BottomRightMotor");
        Collector = hardwareMap.get(DcMotorEx.class, "Collector");
        ShooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        AssistantShooter = hardwareMap.get(DcMotor.class, "AssistantShooter");

        leftOdo = hardwareMap.get(DcMotor.class, "BottomLeftMotor"); // temp, ideally a real encoder
        rightOdo = hardwareMap.get(DcMotor.class, "TopRightMotor");
        mid0do = hardwareMap.get(DcMotor.class, "mid0do"); //mid0do ShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        AssistantShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        mid0do.setDirection(DcMotorSimple.Direction.REVERSE);
        BottomLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BottomRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mid0do.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /* ================= START ===================== */
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /* ================= LOOP ====================== */
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    /* ================= STATE MACHINE ============= */
    private void autonomousPathUpdate() {

        switch (pathState) {

            // ===== PATH 1 =====
            case 0:
                follower.followPath(paths.Path1);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(2);
                }
                break;

            case 2: // 4.8s wait after Path1
                if (pathTimer.getElapsedTime() >= 4.8) {
                    follower.followPath(paths.line2);
                    setPathState(3);
                }
                break;

            // ===== PATH 2 =====
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    setPathState(4);
                }
                break;

            // ===== PATH 3 =====
            case 4:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;

            case 5: // 4.8s wait after Path3
                if (pathTimer.getElapsedTime() >= 4.8) {
                    follower.followPath(paths.Path4);
                    setPathState(6);
                }
                break;

            // ===== PATH 4 =====
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5);
                    setPathState(7);
                }
                break;

            // ===== PATH 5 =====
            case 7:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(8);
                }
                break;

            case 8: // 4.8s wait after Path5
                if (pathTimer.getElapsedTime() >= 4.8) {
                    follower.followPath(paths.Path6);
                    setPathState(9);
                }
                break;

            // ===== PATH 6 =====
            case 9:
                if (!follower.isBusy()) {
                    setPathState(10); // END
                }
                break;
        }
    }

    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    /* ================= PATH DEFINITIONS ========== */
    public static class Paths {

        public PathChain Path1;
        public PathChain line2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(22.726, 126.433),
                            new Pose(23.016, 116.209),
                            new Pose(39.081, 104.804)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(140),
                            Math.toRadians(140))
                    .build();

            line2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(39.081, 104.804),
                            new Pose(49.473, 76.874),
                            new Pose(15.416, 84.407)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(170),
                            Math.toRadians(180))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(15.416, 84.407),
                            new Pose(21.444, 76.699),
                            new Pose(49.739, 93.567)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(140))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(49.739, 93.567),
                            new Pose(54.706, 54.328),
                            new Pose(15.228, 59.592)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(160),
                            Math.toRadians(200))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(15.228, 59.592),
                            new Pose(48.220, 68.650),
                            new Pose(68.053, 77.639)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(200),
                            Math.toRadians(140))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(68.053, 77.639),
                            new Pose(64.606, 89.991),
                            new Pose(105.661, 32.493)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(140),
                            Math.toRadians(140))
                    .build();
        }
    }
}