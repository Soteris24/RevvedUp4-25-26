package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.Arrays;
import java.util.Objects;

public class ArtifactSorter {
    private final RobotHardware hw;
    private final Sorter sorter;
    public boolean motifSortingEnabled = false;

    public boolean artifactPresent = false;
    public String[] storedArtifacts = new String[3];
    public int artifactCount = 0;

    public String[] motif = {"G", "P", "P"};
    public int targetSlot;
    public int nextSlotIndex = 0;
    public int currentSlot = 0;
    public boolean offSetApplied = false;
    int motifProgress = 0;

    private boolean lastButton = false;
    public boolean lastGButton = false;
    public boolean lastPButton = false;

    public boolean baselineAcquired = false;
    private long lastDetectionTime = 0;
    private boolean lastDetected = false;


    int baseline1 = 60;
    int baseline2 = 60;
    Telemetry telemetry;
    PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    boolean telemetryOn;
    public ArtifactSorter(RobotHardware hw, Telemetry telemetry, Sorter sorter, boolean telemetryOn) {
        this.hw = hw;
        this.sorter = sorter;
        this.telemetryOn = telemetryOn;
        this.telemetry = telemetry;
    }

    public void detect2(boolean buttonP, boolean buttonG) {

        if (!baselineAcquired) {
            baseline1 = hw.colorSensor.alpha();
            baseline2 = hw.colorSensor2.alpha();
            baselineAcquired = true;
            return;
        }

        double distance1 = hw.colorSensor.getDistance(DistanceUnit.MM);
        double distance2 = hw.colorSensor2.getDistance(DistanceUnit.MM);

        boolean detectedNow = (distance1 < 25|| distance2 < 25);
        boolean risingEdge = detectedNow && !lastDetected;

        boolean manualTrigger = false;
        String manualColor = null;

        boolean greenPressed = buttonG && !lastGButton;
        boolean purplePressed = buttonP && !lastPButton;

        if (greenPressed) {
            manualTrigger = true;
            manualColor = "G";
        } else if (purplePressed) {
            manualTrigger = true;
            manualColor = "P";
        }

        long now = System.currentTimeMillis();

        long detectionDebounce = 300;
        if (!artifactPresent && (manualTrigger || (risingEdge && now - lastDetectionTime > detectionDebounce))) {

            artifactPresent = true;

            String color;
            if (manualTrigger) {
                color = manualColor;
            } else {
                int green1 = hw.colorSensor.green();
                int blue1 = hw.colorSensor.blue();
                int green2 = hw.colorSensor2.green();
                int blue2 = hw.colorSensor2.blue();

                int maxGreen = Math.max(green1, green2);
                int maxBlue = Math.max(blue1, blue2);

                color = (maxGreen > maxBlue ? "G" : "P");

            }

            if (artifactCount < storedArtifacts.length) {
                storedArtifacts[nextSlotIndex] = color;
                nextSlotIndex = (nextSlotIndex + 1) % 3;
            }
            lastDetectionTime = now;
        }
        lastDetected = detectedNow;
        lastGButton = buttonG;
        lastPButton = buttonP;

        if (telemetryOn) {
            telemetry.addData("RED", Math.max(hw.colorSensor.red(), hw.colorSensor2.red()));
            telemetry.addData("GREEN", Math.max(hw.colorSensor.green(), hw.colorSensor2.green()));
            telemetry.addData("BLUE", Math.max(hw.colorSensor.blue(), hw.colorSensor2.blue()));
            telemetry.addData("BaselineAcquired", baselineAcquired);
            telemetry.addData("baseline1", baseline1);
            telemetry.addData("baseline2", baseline2);
            telemetry.addData("distance1", distance1);
            telemetry.addData("distance2", distance2);
        }
    }


    public void motifSorting(){
        if (!this.motifSortingEnabled) return;
        if (this.artifactCount < 1) return;

        String targetArtifact = motif[motifProgress % motif.length];

        for (int index = 0; index < this.storedArtifacts.length; index++) {
            if (Objects.equals(this.storedArtifacts[index], targetArtifact)) {
                this.targetSlot = index;
                return;
            }
        }

        for (int index = 0; index < this.storedArtifacts.length; index++) {
            if (this.storedArtifacts[index] != null) {
                this.targetSlot = index;
                return;
            }
        }

    }

    public void moveToSlot(int targetSlot) {

        if (storedArtifacts[targetSlot] == null) return;

        if (!offSetApplied) {
            double degreesToMove;
            switch (targetSlot) {
                case 0: degreesToMove = -60;
                    break;
                case 1: degreesToMove = 45;
                    break;
                case 2: degreesToMove = 165;
                    break;
                default: degreesToMove = 0;
            }
            sorter.moveDegrees(degreesToMove);
            offSetApplied = true;
            currentSlot = targetSlot;
            return;
        }
        int delta = (targetSlot -  currentSlot + 3) % 3;
        int step;

        switch (delta){
            case 0: step = 0; break;
            case 1: step = 1; break;
            case 2: step = -1; break;
            default: step = 1; break;
        }

        double degreesToMove = step * RobotHardware.DEG_PER_SLOT;

        if (degreesToMove != 0.0) {
            sorter.moveDegrees(degreesToMove);
        }

        currentSlot = targetSlot;
    }

    public void toggleMotifSorting(boolean button){
        if (button && !lastButton) {
            motifSortingEnabled = !motifSortingEnabled;
        }
        lastButton = button;
    }

    public void pickNextArtifact() {
        if (this.artifactCount < 1) return;
        for (int i = 0; i < storedArtifacts.length; i++) {
            if (storedArtifacts[i] != null) {
                targetSlot = i;
                return;
            }
        }
        targetSlot = -1;

        panelsTelemetry.getTelemetry().addData("Artifact Count", artifactCount);
        panelsTelemetry.getTelemetry().addData("Artifacts", storedArtifacts);
    }
    public void resetSlot() {
        currentSlot = 0;
        artifactCount = 0;
        nextSlotIndex = 0;
        motifProgress = 0;
        offSetApplied = false;
        Arrays.fill(storedArtifacts, null);
    }

    private void addArtifact(String color) {
        if (color == null) return;

        storedArtifacts[nextSlotIndex] = color;
        nextSlotIndex = (nextSlotIndex + 1) % storedArtifacts.length;
    }

    public void removeArtifact(int slot) {
        if (slot < 0 || slot >= storedArtifacts.length) return;
        if (storedArtifacts[slot] != null) {
            storedArtifacts[slot] = null;
            motifProgress = (motifProgress + 1) % motif.length;
        }
    }
}
