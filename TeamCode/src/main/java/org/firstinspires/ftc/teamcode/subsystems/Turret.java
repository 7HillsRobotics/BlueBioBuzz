package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * Subsystem to control turret aiming directly at AprilTags using a continuous rotation servo.
 */
@SuppressWarnings("unused")
public class Turret {

    private final CRServo servo;

    private boolean enabled = true;
    private boolean tracking = false;
    private boolean onTarget = false;
    private double lastTurnErrorDeg = 0.0;
    private int targetTagId = -1; // -1 means track any visible / closest tag

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
            if (servo != null) servo.setPower(0);
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
        if (servo != null) {
            servo.setDirection(inverted ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);
        }
    }

    /**
     * Direct method to point turret toward AprilTags using Vision.
     * @param vision - Vision subsystem
     */
    public void pointToAprilTag(Vision vision) {
        update(vision);
    }

    /**
     * Continuously aims the turret toward detected AprilTags from Vision.
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
            if (servo != null) servo.setPower(0);
            return;
        }

        // TODO: Continuously aim turret toward detected AprilTags.
        // Suggestion: I would get the best AprilTag detection matching targetTagId, calculate the bearing
        // error (degrees) to the tag, and apply proportional power (error * KP + MIN_POWER) to rotate
        // the servo until the error is within TARGET_THRESHOLD_DEG.
        AprilTagDetection bestTag = getAprilTagDetection(detections);

        if (bestTag == null) {
            tracking = false;
            onTarget = false;
            if (servo != null) servo.setPower(0);
        }
    }

    @Nullable
    private AprilTagDetection getAprilTagDetection(List<AprilTagDetection> detections) {
        // TODO: Find the primary AprilTag detection.
        // Suggestion: I would iterate through detections, filter out any that don't match targetTagId (when set),
        // and return the detection with the smallest range/distance.
        return null;
    }

    /**
     * Compatibility wrapper for scoring target status.
     * @param status - scoring target status
     */
    public void update(Vision.ScoringTargetStatus status) {
        if (!enabled || status == null || !status.found) {
            tracking = false;
            onTarget = false;
            if (servo != null) servo.setPower(0);
            return;
        }

        double errorDeg = (status.solution != null) ? status.solution.bearingDeg : 0.0;
        lastTurnErrorDeg = errorDeg;
        tracking = true;

        if (Math.abs(errorDeg) <= TARGET_THRESHOLD_DEG) {
            onTarget = true;
            if (servo != null) servo.setPower(0);
        } else {
            onTarget = false;
            double pPower = errorDeg * KP;
            double power = Math.signum(pPower) * (MIN_POWER + Math.abs(pPower));
            power = Range.clip(power, -1.0, 1.0);
            if (servo != null) servo.setPower(power);
        }
    }

    /**
     * Manual driver override to rotate the continuous servo.
     * @param power - power to apply (-1.0 to 1.0)
     */
    public void manualNudge(double power) {
        tracking = false;
        onTarget = false;
        if (servo != null) {
            servo.setPower(Range.clip(power, -1.0, 1.0));
        }
    }

    /**
     * Safety stop function.
     */
    public void stop() {
        tracking = false;
        onTarget = false;
        if (servo != null) {
            servo.setPower(0);
        }
    }
}
