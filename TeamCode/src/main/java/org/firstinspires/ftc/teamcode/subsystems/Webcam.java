package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Configurable
public class Webcam {
    public AprilTagProcessor tagProcessor;
    public VisionPortal visionPortal;

    public List<AprilTagDetection> detectedTags = new ArrayList<>();

    public Position webcamPosition = new Position(DistanceUnit.INCH, 8.5,0,14.5,0);
    public YawPitchRollAngles webcamOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0,33,0,0);

    private Telemetry telemetry;
    public boolean  blue;

    // -----------RPM CALCULATION VALUES-------------
    public static double closeDistance = 12;
    public static double farDistance = 135;

    public static double closeRPM = 2650; // 3200
    public static double farRPM = 3600;

    public static double closeP = 4;
    public static double farP = 8;
    
    //------------AUTO ALIGNMENT VALUES--------------
    
    public static double turnP = 0.03;
    public static double turnI = 0;
    public static double turnD = 0;
    
    PIDController turnPID = new PIDController(new PIDCoefficients(turnP, turnI, turnD));
    
    public void initTagProcessor(HardwareMap hwMap, Telemetry telemetry, boolean blue) {

        this.telemetry = telemetry;
        this.blue = blue;

        tagProcessor = new AprilTagProcessor.Builder()
                .setCameraPose(webcamPosition, webcamOrientation)
                .setDrawTagID(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera((CameraName) hwMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .addProcessor(tagProcessor)
                .build();
    }

    public void update() {
        detectedTags = tagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags(){
        return detectedTags;
    }

    public AprilTagDetection getLatestTag() {
        if (detectedTags.isEmpty()) return null;
        return detectedTags.get(0);
    }

    public AprilTagDetection getSpecificTag(int id){
        for (AprilTagDetection tag : detectedTags) {
            if (tag.id == id) {
                return tag;
            }
        }
        return null;
    }

    public int getGoalId(){
        if (blue){
            return 20;
        } else return 24;
    }

    public double[] getDistanceAndOffset() {
        AprilTagDetection tag = getSpecificTag(getGoalId());

        if (tag == null) {
            return new double[] {-1,-1};
        }
        return new double[]{
                tag.ftcPose.range,
                tag.ftcPose.bearing
        };
    }

    public boolean farZone() {
        double[] data = getDistanceAndOffset();
        if (data[0] < 0) return false; 
        return data[0] > farDistance;
}

    public double calculateRPM(){
        double RPM;
        double distance = getDistanceAndOffset()[0];

        if (!farZone()){
            double deltaR = distance - closeDistance;
            RPM = closeRPM + deltaR * closeP;
        } else {
            double deltaR = distance - farDistance;
            RPM = farRPM + deltaR * farP;
        }

        return RPM;
    }

    public double getAlignCorrection(double dt) {
        double[] data = getDistanceAndOffset();
        double distance = data[0];
        double bearing = data[1];

        if (distance < 0) return 0;

//        if (distance < 0) {
//           return new double[]{0, 0, 0};
//        }
        bearing = (bearing + 180) % 360 - 180; 

        double turnPower = turnPID.update(bearing, dt);

        return turnPower;
    }
    public String[] getMotif() {
        AprilTagDetection tag = getLatestTag();
        if (tag == null) return null;

        switch (tag.id) {
            case 21: return new String[]{"G", "P", "P"};
            case 22: return new String[]{"P", "G", "P"};
            case 23: return new String[]{"P", "P", "G"};
            default: return null;
        }
    }

    public void updateTelemetry() {
        if (telemetry == null) return;

        String[] motif = getMotif();
        double[] distOffset = getDistanceAndOffset();
        double distance = distOffset[0];
        double rotationOffset = distOffset[1];
        double rpm = calculateRPM();

        telemetry.addData("Target Motif", motif != null ? motif : "UNKNOWN");
        telemetry.addData("Distance to Goal", distance >= 0 ? distance : "N/A");
        telemetry.addData("Calculated RPM", rpm > 0 ? rpm : "N/A");
        telemetry.addData("Rotation Offset", distance >= 0 ? rotationOffset : "N/A");

        telemetry.update();
    }

    public void stop() {
        if (visionPortal != null){
            visionPortal.close();
        }
    }
    public void resetAutoAlign() {
    turnPID.reset();
    }
}