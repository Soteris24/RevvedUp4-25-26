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
    private final Telemetry telemetry;
    private final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    private final boolean telemetryOn;

    public String[] storedArtifacts = new String[3];
    public int artifactCount = 0;
    public int nextSlotIndex = 0;
    public int currentSlot = 0;
    public boolean artifactPresent = false;
    public boolean offSetApplied = false;

    // Motif sorting
    public boolean motifSortingEnabled = false;
    public String[] motif = {"G","P","P"};
    private int motifProgress = 0;
    public int targetSlot = 0;

    // Color detection
    private boolean lastDetected = false;
    private long lastDetectionTime = 0;

    public ArtifactSorter(RobotHardware hw, Telemetry telemetry, Sorter sorter, boolean telemetryOn) {
        this.hw = hw;
        this.sorter = sorter;
        this.telemetry = telemetry;
        this.telemetryOn = telemetryOn;
    }

    public void detect() {
        double d1 = hw.colorSensor.getDistance(DistanceUnit.MM);
        double d2 = hw.colorSensor2.getDistance(DistanceUnit.MM);
        boolean detectedNow = (d1 < 50 || d2 < 50);
        boolean risingEdge = detectedNow && !lastDetected;

        long now = System.currentTimeMillis();
        if (!artifactPresent && risingEdge && now - lastDetectionTime > 300) {
            artifactPresent = true;

            int maxGreen = Math.max(hw.colorSensor.green(), hw.colorSensor2.green());
            int maxBlue  = Math.max(hw.colorSensor.blue(), hw.colorSensor2.blue());
            String color = (maxGreen > maxBlue) ? "G" : "P";

            if (artifactCount < storedArtifacts.length) {
                storedArtifacts[nextSlotIndex] = color;
                moveToSlot(nextSlotIndex);            // rotate sorter immediately
                nextSlotIndex = (nextSlotIndex + 1) % storedArtifacts.length;
                artifactCount++;
            }

            lastDetectionTime = now;
        }

        if (artifactPresent && sorter.atTarget()) {
            artifactPresent = false;                   // ready for next artifact
        }

        lastDetected = detectedNow;

        if (telemetryOn) {
            telemetry.addData("Detected", detectedNow);
            telemetry.addData("ArtifactPresent", artifactPresent);
            telemetry.addData("ArtifactCount", artifactCount);
            telemetry.addData("NextSlotIndex", nextSlotIndex);
            telemetry.addData("StoredArtifacts", Arrays.toString(storedArtifacts));
        }
    }

    public void manualDetect(String color) {
        if (artifactCount >= storedArtifacts.length) return;
        storedArtifacts[nextSlotIndex] = color;
        nextSlotIndex = (nextSlotIndex + 1) % storedArtifacts.length;
        artifactPresent = true;
    }

    public void motifSorting() {
        if (!motifSortingEnabled || artifactCount < 1) return;

        String targetArtifact = motif[motifProgress % motif.length];
        for (int i = 0; i < storedArtifacts.length; i++) {
            if (Objects.equals(storedArtifacts[i], targetArtifact)) {
                targetSlot = i;
                return;
            }
        }
        // fallback
        for (int i = 0; i < storedArtifacts.length; i++) {
            if (storedArtifacts[i] != null) {
                targetSlot = i;
                return;
            }
        }
    }

    public void pickNextArtifact() {
        for (int i = 0; i < storedArtifacts.length; i++) {
            if (storedArtifacts[i] != null) {
                targetSlot = i;
                return;
            }
        }
        targetSlot = -1;
    }

    public void moveToSlot(int slot) {
        if (storedArtifacts[slot] == null) return;

        if (!offSetApplied) {
            double deg;
            switch (slot) {
                case 0: deg = -60; break;
                case 1: deg = 60; break;
                case 2: deg = 180; break;
                default: deg = 0;
            }
            sorter.moveDegrees(deg);
            offSetApplied = true;
            currentSlot = slot;
            return;
        }

        int delta = (slot - currentSlot + 3) % 3;
        int step  = (delta == 0) ? 0 : (delta == 1) ? 1 : -1;
        double deg = step * RobotHardware.DEG_PER_SLOT;
        if (deg != 0.0) sorter.moveDegrees(deg);
        currentSlot = slot;
    }

    public void removeArtifact(int slot) {
        if (slot < 0 || slot >= storedArtifacts.length) return;
        if (storedArtifacts[slot] != null) {
            storedArtifacts[slot] = null;
            motifProgress = (motifProgress + 1) % motif.length;
        }
    }

    public void resetSlot() {
        currentSlot = 0;
        artifactCount = 0;
        nextSlotIndex = 0;
        motifProgress = 0;
        offSetApplied = false;
        Arrays.fill(storedArtifacts, null);
    }
}