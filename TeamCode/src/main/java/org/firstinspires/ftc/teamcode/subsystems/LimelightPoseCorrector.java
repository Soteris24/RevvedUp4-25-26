package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Configurable
public class LimelightPoseCorrector {
    private static final double METERS_TO_INCHES = 39.3700787402;

    public static int APRILTAG_PIPELINE = 0;
    public static int POLL_RATE_HZ = 100;

    public static boolean USE_MT2 = true;
    public static double MAX_RESULT_STALENESS_MS = 100.0;
    public static double MAX_TOTAL_LATENCY_MS = 120.0;
    public static int MIN_TAG_COUNT = 1;
    public static double MIN_TAG_AREA = 0.05;
    public static double MAX_TRANSLATION_STDDEV_METERS = 0.75;
    public static double MAX_HEADING_STDDEV_DEGREES = 45.0;

    public static boolean NEGATE_FIELD_X = false;
    public static boolean NEGATE_FIELD_Y = false;
    public static boolean NEGATE_HEADING = false;
    public static double FIELD_X_OFFSET_IN = 0.0;
    public static double FIELD_Y_OFFSET_IN = 0.0;
    public static double HEADING_OFFSET_DEG = 0.0;

    public static double SNAP_ERROR_IN = 6.0;
    public static double SNAP_ERROR_DEG = 12.0;
    public static double POSITION_BLEND = 0.35;
    public static double HEADING_BLEND = 0.35;

    private final Telemetry telemetry;
    private final Follower follower;
    private final Limelight3A limelight;

    private boolean started;
    private boolean hasVisionLock;
    private Pose lastVisionPose;
    private LLResult lastAcceptedResult;

    public LimelightPoseCorrector(HardwareMap hardwareMap, Follower follower, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.follower = follower;
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public void start() {
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.setPollRateHz(POLL_RATE_HZ);
        limelight.start();
        started = true;
    }

    public void stop() {
        limelight.stop();
        started = false;
    }

    public void update() {
        if (!started) {
            return;
        }

        Pose odomPose = follower.getPose();
        limelight.updateRobotOrientation(Math.toDegrees(odomPose.getHeading()));

        LLResult result = limelight.getLatestResult();
        if (!isAccepted(result)) {
            hasVisionLock = false;
            return;
        }

        Pose measuredPose = toPedroPose(result);
        Pose correctedPose = blendPoses(odomPose, measuredPose);

        follower.setPose(correctedPose);
        hasVisionLock = true;
        lastVisionPose = measuredPose;
        lastAcceptedResult = result;
    }

    public void addTelemetry() {
        telemetry.addData("LL Connected", limelight.isConnected());
        telemetry.addData("LL Running", limelight.isRunning());
        telemetry.addData("LL Vision Lock", hasVisionLock);

        if (lastAcceptedResult != null) {
            telemetry.addData("LL Tags", lastAcceptedResult.getBotposeTagCount());
            telemetry.addData("LL Staleness", "%.0f ms", (double) lastAcceptedResult.getStaleness());
            telemetry.addData("LL Latency", "%.1f ms",
                    lastAcceptedResult.getCaptureLatency() + lastAcceptedResult.getTargetingLatency());
        }

        if (lastVisionPose != null) {
            telemetry.addData("LL Pose", "(%.1f, %.1f, %.1f deg)",
                    lastVisionPose.getX(),
                    lastVisionPose.getY(),
                    Math.toDegrees(lastVisionPose.getHeading()));
        }
    }

    private boolean isAccepted(LLResult result) {
        if (result == null || !result.isValid()) {
            return false;
        }
        if (result.getBotposeTagCount() < MIN_TAG_COUNT) {
            return false;
        }
        if (result.getBotposeAvgArea() < MIN_TAG_AREA) {
            return false;
        }
        if (result.getStaleness() > MAX_RESULT_STALENESS_MS) {
            return false;
        }
        if (result.getCaptureLatency() + result.getTargetingLatency() > MAX_TOTAL_LATENCY_MS) {
            return false;
        }

        double[] stddev = USE_MT2 ? result.getStddevMt2() : result.getStddevMt1();
        if (stddev.length >= 6) {
            double translationStddev = Math.max(Math.abs(stddev[0]), Math.abs(stddev[1]));
            double headingStddev = Math.abs(stddev[5]);
            if (translationStddev > MAX_TRANSLATION_STDDEV_METERS) {
                return false;
            }
            if (headingStddev > MAX_HEADING_STDDEV_DEGREES) {
                return false;
            }
        }

        return true;
    }

    private Pose toPedroPose(LLResult result) {
        Pose3D botpose = USE_MT2 ? result.getBotpose_MT2() : result.getBotpose();
        Position position = botpose.getPosition();
        YawPitchRollAngles orientation = botpose.getOrientation();

        double xInches = position.x * METERS_TO_INCHES;
        double yInches = position.y * METERS_TO_INCHES;
        double headingRadians = Math.toRadians(orientation.getYaw(AngleUnit.DEGREES));

        if (NEGATE_FIELD_X) {
            xInches = -xInches;
        }
        if (NEGATE_FIELD_Y) {
            yInches = -yInches;
        }
        if (NEGATE_HEADING) {
            headingRadians = -headingRadians;
        }

        xInches += FIELD_X_OFFSET_IN;
        yInches += FIELD_Y_OFFSET_IN;
        headingRadians += Math.toRadians(HEADING_OFFSET_DEG);

        return new Pose(xInches, yInches, normalizeRadians(headingRadians));
    }

    private Pose blendPoses(Pose odomPose, Pose measuredPose) {
        double dx = measuredPose.getX() - odomPose.getX();
        double dy = measuredPose.getY() - odomPose.getY();
        double positionError = Math.hypot(dx, dy);
        double headingError = Math.abs(normalizeRadians(measuredPose.getHeading() - odomPose.getHeading()));

        if (!hasVisionLock || positionError >= SNAP_ERROR_IN || Math.toDegrees(headingError) >= SNAP_ERROR_DEG) {
            return measuredPose;
        }

        double blendedX = odomPose.getX() + dx * POSITION_BLEND;
        double blendedY = odomPose.getY() + dy * POSITION_BLEND;
        double blendedHeading = odomPose.getHeading()
                + normalizeRadians(measuredPose.getHeading() - odomPose.getHeading()) * HEADING_BLEND;

        return new Pose(blendedX, blendedY, normalizeRadians(blendedHeading));
    }

    private double normalizeRadians(double angle) {
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }
}
