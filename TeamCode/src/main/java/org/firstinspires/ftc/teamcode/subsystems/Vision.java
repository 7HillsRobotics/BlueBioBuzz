package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;
import android.util.Size;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;

import java.util.ArrayList;
import java.util.List;

/**
 * Complete Vision subsystem for the season.
 * Mounted on the Turret camera portal:
 * 1. AprilTag processing (range, bearing, 3D pose targeting for shooter & turret).
 * 2. Color Blob locator processing (pollen & alliance nectar tracking).
 * 3. Color sensor processing for incoming shooter balls (pollen vs nectar detection).
 */
@SuppressWarnings("unused")
public class Vision {

    private final VisionPortal turretPortal;
    private final AprilTagProcessor turretAprilTag;
    private final ColorBlobLocatorProcessor pollenLocator;
    private final ColorBlobLocatorProcessor nectarLocatorRed;
    private final ColorBlobLocatorProcessor nectarLocatorBlue;

    private final VisionPortal chassisPortal;
    private final AprilTagProcessor chassisAprilTag;

    private final NormalizedColorSensor ballColorSensor;
    private Ball lastDetectedBall = Ball.NONE;

    public Vision(HardwareMap hw) {
        // --- Processors Setup ---
        turretAprilTag = new AprilTagProcessor.Builder().build();
        chassisAprilTag = new AprilTagProcessor.Builder().build();

        pollenLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .build();

        nectarLocatorRed = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .build();

        nectarLocatorBlue = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .build();

        // --- Hardware Lookups ---
        NormalizedColorSensor colorSensor = null;
        try {
            colorSensor = hw.get(NormalizedColorSensor.class, "ballColor");
        } catch (Exception ignored) {}
        this.ballColorSensor = colorSensor;

        WebcamName turretCam = null;
        WebcamName chassisCam = null;

        try {
            turretCam = hw.get(WebcamName.class, "Webcam Turret");
        } catch (Exception ignored) {}

        try {
            chassisCam = hw.get(WebcamName.class, "Webcam Chassis");
        } catch (Exception ignored) {}

        // Fallback to "Webcam 1" as primary turret camera if name is standard
        if (turretCam == null && chassisCam == null) {
            try {
                turretCam = hw.get(WebcamName.class, "Webcam 1");
            } catch (Exception ignored) {}
        }

        // --- Camera Portal Initialization ---
        if (turretCam != null && chassisCam != null) {
            int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

            // Turret camera gets AprilTag + Color Blob Locators
            turretPortal = new VisionPortal.Builder()
                    .setCamera(turretCam)
                    .setCameraResolution(new Size(640, 480))
                    .addProcessor(turretAprilTag)
                    .addProcessor(pollenLocator)
                    .addProcessor(nectarLocatorRed)
                    .addProcessor(nectarLocatorBlue)
                    .setLiveViewContainerId(viewIds[0])
                    .build();

            chassisPortal = new VisionPortal.Builder()
                    .setCamera(chassisCam)
                    .setCameraResolution(new Size(640, 480))
                    .addProcessor(chassisAprilTag)
                    .setLiveViewContainerId(viewIds[1])
                    .build();
        } else if (turretCam != null) {
            // Single Turret Camera mode (Standard setup)
            turretPortal = new VisionPortal.Builder()
                    .setCamera(turretCam)
                    .setCameraResolution(new Size(640, 480))
                    .addProcessor(turretAprilTag)
                    .addProcessor(pollenLocator)
                    .addProcessor(nectarLocatorRed)
                    .addProcessor(nectarLocatorBlue)
                    .build();
            chassisPortal = null;
        } else if (chassisCam != null) {
            // Fallback Chassis Camera mode
            turretPortal = null;
            chassisPortal = new VisionPortal.Builder()
                    .setCamera(chassisCam)
                    .setCameraResolution(new Size(640, 480))
                    .addProcessor(chassisAprilTag)
                    .addProcessor(pollenLocator)
                    .addProcessor(nectarLocatorRed)
                    .addProcessor(nectarLocatorBlue)
                    .build();
        } else {
            turretPortal = null;
            chassisPortal = null;
        }
    }

    // ==========================================
    // --- BALL COLOR SENSOR (Shooter Queue) ---
    // ==========================================

    /**
     * Reads the color sensor mounted on the path to the shooter.
     * Detects incoming ball type (pollen vs red/blue nectar) and updates state.
     */
    public void updateBallSensor() {
        if (ballColorSensor == null) return;

        NormalizedRGBA colors = ballColorSensor.getNormalizedColors();
        float intensity = colors.red + colors.green + colors.blue;

        if (intensity > 0.05f) {
            float[] hsv = new float[3];
            Color.RGBToHSV(
                    Math.min(255, (int)(colors.red * 255)),
                    Math.min(255, (int)(colors.green * 255)),
                    Math.min(255, (int)(colors.blue * 255)),
                    hsv
            );

            float hue = hsv[0];
            float saturation = hsv[1];
            float value = hsv[2];

            if (saturation > 0.2f && value > 0.1f) {
                if (hue >= 35 && hue <= 90) {
                    lastDetectedBall = Ball.POLLEN;       // Yellow pollen ball
                } else if (hue >= 180 && hue <= 250) {
                    lastDetectedBall = Ball.NECTAR_BLUE;  // Blue nectar ball
                } else if (hue < 30 || hue > 330) {
                    lastDetectedBall = Ball.NECTAR_RED;   // Red nectar ball
                }
            }
        }
    }

    public Ball getLastDetectedBall() {
        return lastDetectedBall;
    }

    public Ball getNextBallForShooter() {
        return lastDetectedBall;
    }

    public ShooterMode getShooterModeForNextBall() {
        if (lastDetectedBall == Ball.POLLEN) {
            return ShooterMode.POLLEN;
        } else if (lastDetectedBall == Ball.NECTAR_RED || lastDetectedBall == Ball.NECTAR_BLUE) {
            return ShooterMode.NECTAR;
        }
        return ShooterMode.NONE;
    }

    // ==========================================
    // --- APRILTAG DETECTIONS & CALCULATIONS ---
    // ==========================================

    public static int getTagId(AprilTagDetection tag) {
        if (tag instanceof AprilTagSingleDetection) {
            return ((AprilTagSingleDetection) tag).id;
        }
        return -1;
    }

    public List<AprilTagDetection> getTurretAprilTags() {
        if (turretAprilTag != null) {
            return turretAprilTag.getDetections();
        }
        return new ArrayList<>();
    }

    public List<AprilTagDetection> getChassisAprilTags() {
        if (chassisAprilTag != null) {
            return chassisAprilTag.getDetections();
        }
        return new ArrayList<>();
    }

    public List<AprilTagDetection> getAllAprilTags() {
        List<AprilTagDetection> tags = new ArrayList<>();
        tags.addAll(getTurretAprilTags());
        tags.addAll(getChassisAprilTags());
        return tags;
    }

    @Nullable
    public AprilTagDetection getClosestAprilTag() {
        List<AprilTagDetection> detections = getAllAprilTags();
        if (detections.isEmpty()) return null;

        AprilTagDetection closest = null;
        double minDistance = Double.MAX_VALUE;

        for (AprilTagDetection tag : detections) {
            if (tag == null || tag.ftcPose == null) continue;
            if (tag.ftcPose.range < minDistance) {
                minDistance = tag.ftcPose.range;
                closest = tag;
            }
        }
        return closest;
    }

    @Nullable
    public AprilTagDetection getAprilTagById(int targetId) {
        List<AprilTagDetection> detections = getAllAprilTags();
        for (AprilTagDetection tag : detections) {
            if (tag != null && getTagId(tag) == targetId) {
                return tag;
            }
        }
        return null;
    }

    public double getAprilTagRangeInches() {
        AprilTagDetection closest = getClosestAprilTag();
        if (closest != null && closest.ftcPose != null) {
            return closest.ftcPose.range;
        }
        return -1.0;
    }

    public double getAprilTagBearingDegrees() {
        AprilTagDetection closest = getClosestAprilTag();
        if (closest != null && closest.ftcPose != null) {
            return closest.ftcPose.bearing;
        }
        return 0.0;
    }

    @Nullable
    public AprilTagPoseFtc getClosestAprilTagPose() {
        AprilTagDetection closest = getClosestAprilTag();
        return (closest != null) ? closest.ftcPose : null;
    }

    /**
     * Set decimation on turret AprilTag processor to adjust speed vs resolution tradeoff.
     * @param decimation - decimation factor (e.g. 1.0f - 3.0f)
     */
    public void setTurretAprilTagDecimation(float decimation) {
        if (turretAprilTag != null) {
            turretAprilTag.setDecimation(decimation);
        }
    }

    // ==========================================
    // --- COLOR BLOB DETECTIONS (Pollen/Nectar) ---
    // ==========================================

    public List<ColorBlobLocatorProcessor.Blob> getPollenBlobs() {
        if (pollenLocator != null) {
            return pollenLocator.getBlobs();
        }
        return new ArrayList<>();
    }

    public List<ColorBlobLocatorProcessor.Blob> getRedNectarBlobs() {
        if (nectarLocatorRed != null) {
            return nectarLocatorRed.getBlobs();
        }
        return new ArrayList<>();
    }

    public List<ColorBlobLocatorProcessor.Blob> getBlueNectarBlobs() {
        if (nectarLocatorBlue != null) {
            return nectarLocatorBlue.getBlobs();
        }
        return new ArrayList<>();
    }

    public List<ColorBlobLocatorProcessor.Blob> getNectarBlobs(AllianceColor alliance) {
        return (alliance == AllianceColor.RED) ? getRedNectarBlobs() : getBlueNectarBlobs();
    }

    @Nullable
    public ColorBlobLocatorProcessor.Blob getLargestBlob(List<ColorBlobLocatorProcessor.Blob> blobs) {
        if (blobs == null || blobs.isEmpty()) return null;
        ColorBlobLocatorProcessor.Blob largest = blobs.get(0);
        for (ColorBlobLocatorProcessor.Blob blob : blobs) {
            if (blob != null && blob.getContourArea() > largest.getContourArea()) {
                largest = blob;
            }
        }
        return largest;
    }

    @Nullable
    public ColorBlobLocatorProcessor.Blob getBestPollenBlob() {
        return getLargestBlob(getPollenBlobs());
    }

    @Nullable
    public ColorBlobLocatorProcessor.Blob getBestNectarBlob(AllianceColor alliance) {
        return getLargestBlob(getNectarBlobs(alliance));
    }

    // ==========================================
    // --- PORTAL & STREAMING CONTROLS ---
    // ==========================================

    public boolean isTurretCameraReady() {
        return turretPortal != null && turretPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    public boolean isChassisCameraReady() {
        return chassisPortal != null && chassisPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    public void setTurretAprilTagEnabled(boolean enabled) {
        if (turretPortal != null && turretAprilTag != null) {
            turretPortal.setProcessorEnabled(turretAprilTag, enabled);
        }
    }

    public void setPollenLocatorEnabled(boolean enabled) {
        if (turretPortal != null && pollenLocator != null) {
            turretPortal.setProcessorEnabled(pollenLocator, enabled);
        } else if (chassisPortal != null && pollenLocator != null) {
            chassisPortal.setProcessorEnabled(pollenLocator, enabled);
        }
    }

    public void setNectarLocatorEnabled(boolean enabled) {
        if (turretPortal != null) {
            if (nectarLocatorRed != null) turretPortal.setProcessorEnabled(nectarLocatorRed, enabled);
            if (nectarLocatorBlue != null) turretPortal.setProcessorEnabled(nectarLocatorBlue, enabled);
        } else if (chassisPortal != null) {
            if (nectarLocatorRed != null) chassisPortal.setProcessorEnabled(nectarLocatorRed, enabled);
            if (nectarLocatorBlue != null) chassisPortal.setProcessorEnabled(nectarLocatorBlue, enabled);
        }
    }

    public void stopStreaming() {
        if (turretPortal != null) turretPortal.stopStreaming();
        if (chassisPortal != null) chassisPortal.stopStreaming();
    }

    public void resumeStreaming() {
        if (turretPortal != null) turretPortal.resumeStreaming();
        if (chassisPortal != null) chassisPortal.resumeStreaming();
    }

    public void close() {
        if (turretPortal != null) turretPortal.close();
        if (chassisPortal != null) chassisPortal.close();
    }

    // ==========================================
    // --- AUXILIARY TYPES & STUBS ---
    // ==========================================

    public enum ShooterMode { NONE, POLLEN, NECTAR }
    public enum CameraSource { TURRET_CAM, CHASSIS_CAM }
    public enum AllianceColor { RED, BLUE }

    public static class TargetSolution {
        public final CameraSource source;
        public final String clusterName;
        public final double bearingDeg;
        public final double rangeIn;

        public TargetSolution(CameraSource source, String clusterName, double bearingDeg, double rangeIn) {
            this.source = source;
            this.clusterName = clusterName;
            this.bearingDeg = bearingDeg;
            this.rangeIn = rangeIn;
        }
    }

    public static class ScoringTargetStatus {
        public final boolean found;
        public final TargetSolution solution;

        public ScoringTargetStatus(boolean found, TargetSolution solution) {
            this.found = found;
            this.solution = solution;
        }
    }

    public ScoringTargetStatus getScorableTarget(AllianceColor alliance) {
        AprilTagDetection closest = getClosestAprilTag();
        if (closest != null && closest.ftcPose != null) {
            TargetSolution sol = new TargetSolution(CameraSource.TURRET_CAM, "AprilTag_" + getTagId(closest),
                    closest.ftcPose.bearing, closest.ftcPose.range);
            return new ScoringTargetStatus(true, sol);
        }
        return new ScoringTargetStatus(false, null);
    }

    public enum Ball {
        NONE, POLLEN, NECTAR_RED, NECTAR_BLUE, UNKNOWN;

        public boolean isLarge() {
            return this == NECTAR_RED || this == NECTAR_BLUE || this == UNKNOWN;
        }

        public boolean isPollen() {
            return this == POLLEN;
        }

        public boolean isNectar() {
            return this == NECTAR_RED || this == NECTAR_BLUE;
        }
    }

    public Ball getCurrentBall() {
        return lastDetectedBall;
    }
}
