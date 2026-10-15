package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Subsystem to control the shooter flywheel and hood angle.
 * Target flywheel speed should be calculated based on robot pose, goal location,
 * or vision AprilTag distance.
 */
@SuppressWarnings("unused")
public class Shooter {

    private final DcMotorEx motor;
    private final Servo hood;

    private boolean spinning = false;
    private double targetRpm = 0.0;
    private double hoodPosition = 0.0;

    // Motor hardware constants (GoBilda Yellowjacket 1:1 or REV HD Hex flywheel motor)
    public static double TICKS_PER_REV = 28.0; 
    public static double RPM_TOLERANCE = 50.0; // Allowed error margin for isAtSpeed()

    // Default Goal location on the FTC field (in inches, relative to field origin)
    private Vector2d goalPosition = new Vector2d(72.0, 72.0);

    // Speed calculation curve coefficients: RPM = A * d^2 + B * d + C
    public static double RPM_COEFF_A = 0.05;
    public static double RPM_COEFF_B = 18.0;
    public static double RPM_COEFF_C = 1600.0;
    public static double MIN_RPM = 1500.0;
    public static double MAX_RPM = 5500.0;

    public static double HOOD_SMALL = 0.20;
    public static double HOOD_LARGE = 0.80;

    public enum Mode {
        POLLEN, NECTAR, DEFAULT
    }

    private Mode currentMode = Mode.DEFAULT;

    public Shooter(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "shooter");
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        Servo tempHood = null;
        try {
            tempHood = hw.get(Servo.class, "hood");
        } catch (Exception ignored) {}
        this.hood = tempHood;
    }

    /**
     * Turn the flywheel on or off
     * @param on - spinning state
     */
    public void setSpinning(boolean on) {
        spinning = on;
        if (!on && motor != null) {
            motor.setPower(0);
        }
    }

    public boolean isSpinning() {
        return spinning;
    }

    /**
     * Set target flywheel RPM directly
     * @param rpm - target RPM
     */
    public void setTargetRpm(double rpm) {
        this.targetRpm = Math.max(0.0, Math.min(MAX_RPM, rpm));
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    /**
     * Set target RPM based on range/distance to target in inches
     * @param rangeInches - distance to target in inches
     * @return calculated target RPM
     */
    public double setTargetRpmForRange(double rangeInches) {
        double rpm = calculatePerfectRpm(rangeInches);
        setTargetRpm(rpm);
        return rpm;
    }

    /**
     * Calculates the optimal flywheel RPM for a given distance in inches using
     * the distance-to-RPM speed curve.
     * @param distanceInches - distance to target goal in inches
     * @return calculated target RPM
     */
    public double calculatePerfectRpm(double distanceInches) {
        // TODO: Calculate ideal flywheel RPM based on distance to the target goal in inches.
        // Suggestion: I would use a distance-to-RPM curve (such as a quadratic equation: RPM = A*d^2 + B*d + C)
        // and clamp the final RPM between MIN_RPM and MAX_RPM.
        if (distanceInches <= 0) return MIN_RPM;

        return MIN_RPM;
    }

    /**
     * Calculates the distance from the robot's current pose to the target goal,
     * and sets the ideal flywheel RPM automatically.
     * @param botPose - current robot field position (Pose2d)
     * @param goal - target goal location on field (Vector2d)
     * @return calculated target RPM
     */
    public double setTargetRpmFromPosition(Pose2d botPose, Vector2d goal) {
        // TODO: Calculate distance from botPose to goal position and set target RPM.
        // Suggestion: I would calculate the Euclidean distance (Math.hypot) between botPose and goal position,
        // and then pass that distance to setTargetRpmForRange().
        if (botPose == null || goal == null) return targetRpm;

        return targetRpm;
    }

    /**
     * Calculates the distance from the robot's current pose to the default goal position,
     * and sets the ideal flywheel RPM automatically.
     * @param botPose - current robot field position (Pose2d)
     * @return calculated target RPM
     */
    public double setTargetRpmFromPosition(Pose2d botPose) {
        return setTargetRpmFromPosition(botPose, this.goalPosition);
    }

    /**
     * Uses detected AprilTag range in Vision to set the ideal flywheel RPM.
     * @param vision - Vision subsystem
     * @return calculated target RPM
     */
    public double setTargetRpmFromVision(Vision vision) {
        if (vision == null) return targetRpm;

        double rangeInches = vision.getAprilTagRangeInches();
        if (rangeInches > 0) {
            return setTargetRpmForRange(rangeInches);
        }
        return targetRpm;
    }

    /**
     * Sets the global goal position coordinates on the field.
     * @param goal - Vector2d goal location
     */
    public void setGoalPosition(Vector2d goal) {
        if (goal != null) {
            this.goalPosition = goal;
        }
    }

    public Vector2d getGoalPosition() {
        return goalPosition;
    }

    /**
     * Get the current measured flywheel RPM from motor encoder
     */
    public double getRpm() {
        // TODO: Read actual flywheel velocity from motor encoder.
        // Suggestion: I would read motor.getVelocity() (encoder ticks/sec), then convert ticks/sec to RPM
        // using TICKS_PER_REV and multiplying by 60.
        if (motor == null) return 0.0;

        return 0.0;
    }

    /**
     * Returns whether the flywheel is at target speed within tolerance
     */
    public boolean isAtSpeed() {
        // TODO: Check if flywheel has reached target speed.
        // Suggestion: I would check if spinning is true and Math.abs(getRpm() - targetRpm) <= RPM_TOLERANCE.
        return spinning;
    }

    public void setHoodPosition(double position) {
        this.hoodPosition = position;
        if (hood != null) {
            hood.setPosition(position);
        }
    }

    public double getHoodPosition() {
        return hoodPosition;
    }

    public void setMode(Mode mode) {
        this.currentMode = mode;
        if (mode == Mode.POLLEN) {
            setHoodPosition(HOOD_SMALL);
        } else if (mode == Mode.NECTAR) {
            setHoodPosition(HOOD_LARGE);
        }
    }

    public Mode getMode() {
        return currentMode;
    }

    public void setHoodForBall(Vision.Ball ball) {
        if (ball == Vision.Ball.NONE) return;
        setHoodPosition(ball.isLarge() ? HOOD_LARGE : HOOD_SMALL);
    }

    public void setHoodFromVision(Vision vision) {
        if (vision == null) return;
        vision.updateBallSensor();
        setHoodForBall(vision.getNextBallForShooter());
    }

    public void updateModeFromVision(Vision vision) {
        if (vision == null) return;
        vision.updateBallSensor();
        Vision.ShooterMode shooterMode = vision.getShooterModeForNextBall();
        if (shooterMode == Vision.ShooterMode.POLLEN) {
            setMode(Mode.POLLEN);
        } else if (shooterMode == Vision.ShooterMode.NECTAR) {
            setMode(Mode.NECTAR);
        }
    }

    /**
     * Runs the flywheel closed-loop velocity control loop.
     * Call this inside your main Opmode loop!
     */
    public void update() {
        // TODO: Run flywheel velocity control loop.
        // Suggestion: I would check if spinning is true and targetRpm > 0, then convert targetRpm
        // into target ticks/sec and call motor.setVelocity(targetTicksPerSec).
        if (!spinning || targetRpm <= 0) {
            if (motor != null) motor.setPower(0);
        }
    }

    /**
     * Safety stop function
     */
    public void stop() {
        spinning = false;
        if (motor != null) {
            motor.setPower(0);
        }
    }
}
