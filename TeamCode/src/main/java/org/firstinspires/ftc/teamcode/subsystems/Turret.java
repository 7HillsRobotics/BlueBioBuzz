package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

/**
 * Class to control turret aiming toward AprilTags using a continuously rotating servo.
 */
@SuppressWarnings("unused")
public class Turret {

    private final CRServo servo;

    private boolean enabled = true;
    private boolean tracking = false;
    private boolean onTarget = false;
    private double lastTurnErrorDeg = 0.0;
    private int targetTagId = -1; // -1 means track any visible/closest tag

    public static double KP = 0.025;
    public static double MIN_POWER = 0.08;
    public static double TARGET_THRESHOLD_DEG = 1.5;

    public Turret(HardwareMap hw) {
        servo = hw.get(CRServo.class, "turret");
    }

    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Enable or disable turret tracking.
     * @param e - enabled state
     */
    public void setEnabled(boolean e) {
        enabled = e;
        if (!e) {
            tracking = false;
            onTarget = false;
            servo.setPower(0);
        }
    }

    public boolean isTracking() {
        return tracking;
    }

    public boolean isOnTarget() {
        return tracking && onTarget;
    }

    public double getLastTurnErrorDeg() {
        return lastTurnErrorDeg;
    }

    public void setTargetTagId(int tagId) {
        this.targetTagId = tagId;
    }

    public int getTargetTagId() {
        return targetTagId;
    }

    /**
     * Inverts the servo rotation direction if mounted backwards.
     */
    public void setInverted(boolean inverted) {
        servo.setDirection(inverted ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);
    }

    /**
     * Continuously aims the turret toward detected AprilTags in Vision.
     * @param vision - Vision subsystem
     */
    public void update(Vision vision) {
        if (vision == null) {
            stop();
            return;
        }
        List<AprilTagDetection> detections = vision.getTurretAprilTags();
        if (detections.isEmpty()) {
            detections = vision.getAllAprilTags();
        }
        update(detections);
    }

    /**
     * Continuously aims the turret toward detected AprilTags.
     * @param detections - List of detected AprilTags
     */
    public void update(List<AprilTagDetection> detections) {
        if (!enabled || detections == null || detections.isEmpty()) {
            tracking = false;
            onTarget = false;
            servo.setPower(0);
            return;
        }

        AprilTagDetection bestTag = getAprilTagDetection(detections);

        if (bestTag == null) {
            tracking = false;
            onTarget = false;
            servo.setPower(0);
            return;
        }

        // Calculate bearing error to the target tag in degrees
        double errorDeg = 0.0;
        if (bestTag.ftcPose != null) {
            errorDeg = bestTag.ftcPose.bearing;
        } else if (bestTag instanceof AprilTagSingleDetection) {
            AprilTagSingleDetection singleDet = (AprilTagSingleDetection) bestTag;
            if (singleDet.center != null) {
                errorDeg = (singleDet.center.x - 320.0) / 10.0;
            }
        }

        lastTurnErrorDeg = errorDeg;
        tracking = true;

        if (Math.abs(errorDeg) <= TARGET_THRESHOLD_DEG) {
            onTarget = true;
            servo.setPower(0);
        } else {
            onTarget = false;
            double pPower = errorDeg * KP;
            double power = Math.signum(pPower) * (MIN_POWER + Math.abs(pPower));
            power = Range.clip(power, -1.0, 1.0);
            servo.setPower(power);
        }
    }

    @Nullable
    private AprilTagDetection getAprilTagDetection(List<AprilTagDetection> detections) {
        AprilTagDetection bestTag = null;
        double minDistance = Double.MAX_VALUE;

        for (AprilTagDetection detection : detections) {
            if (detection == null) continue;

            // Filter by specific tag ID if set
            if (targetTagId != -1) {
                if (detection instanceof AprilTagSingleDetection) {
                    AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;
                    if (singleDet.id != targetTagId) continue;
                } else {
                    continue;
                }
            }

            double dist = (detection.ftcPose != null) ? detection.ftcPose.range : 100.0;
            if (dist < minDistance) {
                minDistance = dist;
                bestTag = detection;
            }
        }
        return bestTag;
    }

    /**
     * Compatibility wrapper for scoring target status.
     * @param status - scoring target status
     */
    public void update(Vision.ScoringTargetStatus status) {
        if (!enabled || status == null || !status.found) {
            tracking = false;
            onTarget = false;
            servo.setPower(0);
            return;
        }

        double errorDeg = (status.solution != null) ? status.solution.bearingDeg : 0.0;
        lastTurnErrorDeg = errorDeg;
        tracking = true;

        if (Math.abs(errorDeg) <= TARGET_THRESHOLD_DEG) {
            onTarget = true;
            servo.setPower(0);
        } else {
            onTarget = false;
            double pPower = errorDeg * KP;
            double power = Math.signum(pPower) * (MIN_POWER + Math.abs(pPower));
            power = Range.clip(power, -1.0, 1.0);
            servo.setPower(power);
        }
    }

    /**
     * Manual driver override to rotate the continuous servo.
     * @param power - power to apply (-1.0 to 1.0)
     */
    public void manualNudge(double power) {
        tracking = false;
        onTarget = false;
        servo.setPower(Range.clip(power, -1.0, 1.0));
    }

    /**
     * Safety stop function.
     */
    public void stop() {
        tracking = false;
        onTarget = false;
        servo.setPower(0);
    }
}
