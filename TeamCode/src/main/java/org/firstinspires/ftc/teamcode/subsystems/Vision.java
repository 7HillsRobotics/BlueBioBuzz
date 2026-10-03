package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;

import java.util.ArrayList;
import java.util.List;

/**
 * Vision subsystem for AprilTag detection, finding pollen & nectar color blobs,
 * and detecting the next ball entering the shooter via color sensor.
 */
@SuppressWarnings("unused")
public class Vision {

    private final VisionPortal turretPortal;
    private final AprilTagProcessor turretAprilTag;

    private final VisionPortal chassisPortal;
    private final AprilTagProcessor chassisAprilTag;
    private final ColorBlobLocatorProcessor pollenLocator;
    private final ColorBlobLocatorProcessor nectarLocatorRed;
    private final ColorBlobLocatorProcessor nectarLocatorBlue;

    private final NormalizedColorSensor ballColorSensor;
    private Ball lastDetectedBall = Ball.NONE;

    public Vision(HardwareMap hw) {
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

        if (turretCam == null && chassisCam == null) {
            try {
                chassisCam = hw.get(WebcamName.class, "Webcam 1");
            } catch (Exception ignored) {}
        }

        if (turretCam != null && chassisCam != null) {
            int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

            turretPortal = new VisionPortal.Builder()
                    .setCamera(turretCam)
                    .setCameraResolution(new Size(640, 480))
                    .addProcessor(turretAprilTag)
                    .setLiveViewContainerId(viewIds[0])
                    .build();

            chassisPortal = new VisionPortal.Builder()
                    .setCamera(chassisCam)
                    .setCameraResolution(new Size(640, 480))
                    .addProcessor(chassisAprilTag)
                    .addProcessor(pollenLocator)
                    .addProcessor(nectarLocatorRed)
                    .addProcessor(nectarLocatorBlue)
                    .setLiveViewContainerId(viewIds[1])
                    .build();
        } else if (turretCam != null) {
            turretPortal = new VisionPortal.Builder()
                    .setCamera(turretCam)
                    .setCameraResolution(new Size(640, 480))
                    .addProcessor(turretAprilTag)
                    .build();
            chassisPortal = null;
        } else if (chassisCam != null) {
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

    // --- Ball Sensor Detection (Next Ball for Shooter) ---

    /**
     * Reads the color sensor mounted on the path to the shooter.
     * Detects the next ball heading into the shooter and records its type (pollen vs nectar).
     */
    public void updateBallSensor() {
        if (ballColorSensor == null) return;

        NormalizedRGBA colors = ballColorSensor.getNormalizedColors();
        float intensity = colors.red + colors.green + colors.blue;

        // Trigger detection if a ball is sufficiently close
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
                    lastDetectedBall = Ball.POLLEN;       // Yellow ball
                } else if (hue >= 180 && hue <= 250) {
                    lastDetectedBall = Ball.NECTAR_BLUE;  // Blue nectar ball
                } else if (hue < 30 || hue > 330) {
                    lastDetectedBall = Ball.NECTAR_RED;   // Red nectar ball
                }
            }
        }
    }

    /**
     * Gets the last ball detected by the color sensor heading to the shooter.
     */
    public Ball getLastDetectedBall() {
        return lastDetectedBall;
    }

    /**
     * Alias for getLastDetectedBall(). Represents the next ball entering the shooter.
     */
    public Ball getNextBallForShooter() {
        return lastDetectedBall;
    }

    /**
     * Gets the target shooter mode (POLLEN vs NECTAR) based on the next ball detected.
     */
    public ShooterMode getShooterModeForNextBall() {
        if (lastDetectedBall == Ball.POLLEN) {
            return ShooterMode.POLLEN;
        } else if (lastDetectedBall == Ball.NECTAR_RED || lastDetectedBall == Ball.NECTAR_BLUE) {
            return ShooterMode.NECTAR;
        }
        return ShooterMode.NONE;
    }

    // --- AprilTag Detections ---

    /**
     * Gets detected AprilTags from the turret camera.
     */
    public List<AprilTagDetection> getTurretAprilTags() {
        if (turretAprilTag != null) {
            return turretAprilTag.getDetections();
        }
        return new ArrayList<>();
    }

    /**
     * Gets detected AprilTags from the chassis camera.
     */
    public List<AprilTagDetection> getChassisAprilTags() {
        if (chassisAprilTag != null) {
            return chassisAprilTag.getDetections();
        }
        return new ArrayList<>();
    }

    /**
     * Gets all detected AprilTags across all active cameras.
     */
    public List<AprilTagDetection> getAllAprilTags() {
        List<AprilTagDetection> tags = new ArrayList<>();
        tags.addAll(getTurretAprilTags());
        tags.addAll(getChassisAprilTags());
        return tags;
    }

    // --- Pollen & Nectar Color Blob Detections ---

    /**
     * Gets detected pollen (yellow) blobs from the chassis camera.
     */
    public List<ColorBlobLocatorProcessor.Blob> getPollenBlobs() {
        if (pollenLocator != null) {
            return pollenLocator.getBlobs();
        }
        return new ArrayList<>();
    }

    /**
     * Gets detected red nectar blobs from the chassis camera.
     */
    public List<ColorBlobLocatorProcessor.Blob> getRedNectarBlobs() {
        if (nectarLocatorRed != null) {
            return nectarLocatorRed.getBlobs();
        }
        return new ArrayList<>();
    }

    /**
     * Gets detected blue nectar blobs from the chassis camera.
     */
    public List<ColorBlobLocatorProcessor.Blob> getBlueNectarBlobs() {
        if (nectarLocatorBlue != null) {
            return nectarLocatorBlue.getBlobs();
        }
        return new ArrayList<>();
    }

    /**
     * Gets detected nectar blobs for the specified alliance color.
     */
    public List<ColorBlobLocatorProcessor.Blob> getNectarBlobs(AllianceColor alliance) {
        return (alliance == AllianceColor.RED) ? getRedNectarBlobs() : getBlueNectarBlobs();
    }

    // --- Portal Status & Control ---

    public boolean isTurretCameraReady() {
        return turretPortal != null && turretPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    public boolean isChassisCameraReady() {
        return chassisPortal != null && chassisPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    public void setTurretCamAprilTagEnabled(boolean enabled) {
        if (turretPortal != null && turretAprilTag != null) {
            turretPortal.setProcessorEnabled(turretAprilTag, enabled);
        }
    }

    public void setChassisCamAprilTagEnabled(boolean enabled) {
        if (chassisPortal != null && chassisAprilTag != null) {
            chassisPortal.setProcessorEnabled(chassisAprilTag, enabled);
        }
    }

    public void setPollenLocatorEnabled(boolean enabled) {
        if (chassisPortal != null && pollenLocator != null) {
            chassisPortal.setProcessorEnabled(pollenLocator, enabled);
        }
    }

    public void setNectarLocatorEnabled(boolean enabled) {
        if (chassisPortal != null) {
            if (nectarLocatorRed != null) chassisPortal.setProcessorEnabled(nectarLocatorRed, enabled);
            if (nectarLocatorBlue != null) chassisPortal.setProcessorEnabled(nectarLocatorBlue, enabled);
        }
    }

    public void close() {
        if (turretPortal != null) turretPortal.close();
        if (chassisPortal != null) chassisPortal.close();
    }

    // --- Auxiliary Types & Stubs ---

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
