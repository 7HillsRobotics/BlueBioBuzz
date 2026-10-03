package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Class to control the shooter flywheel and hood
 */
@SuppressWarnings("unused")
public class Shooter {

    // declare hardware here
    private final DcMotorEx motor;
    private final Servo hood;

    private boolean spinning = false;
    private double targetRpm = 0.0;
    private double hoodPosition = 0.0;

    public Shooter(HardwareMap hw) {
        // initialize hardware here
        motor = hw.get(DcMotorEx.class, "shooter");
        hood = hw.get(Servo.class, "hood");
    }

    /**
     * Turn the flywheel on or off
     * @param on - spinning state
     */
    public void setSpinning(boolean on) {
        spinning = on;
        if (!on) motor.setPower(0);
    }

    public boolean isSpinning() { return spinning; }

    public void setTargetRpm(double rpm) {
        targetRpm = rpm;
    }

    /**
     * Set target RPM based on range to target
     * @param rangeIn - range to target, in inches
     */
    public void setTargetRpmForRange(double rangeIn) {
        // range-to-rpm lookup here
    }

    public double getTargetRpm() { return targetRpm; }

    /**
     * Get the current flywheel RPM
     */
    public double getRpm() {
        return 0.0; // TODO: read from motor encoder
    }

    public boolean isAtSpeed() {
        return false;
    }

    public void setHoodPosition(double position) {
        hoodPosition = position;
        hood.setPosition(position);
    }

    public double getHoodPosition() { return hoodPosition; }

    public static double HOOD_SMALL = 0.20;
    public static double HOOD_LARGE = 0.80;

    public enum Mode {
        POLLEN, NECTAR, DEFAULT
    }

    private Mode currentMode = Mode.DEFAULT;

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

    /**
     * Set hood position based on the ball currently headed into the turret
     * @param ball - ball identified by the color sensor
     */
    public void setHoodForBall(Vision.Ball ball) {
        if (ball == Vision.Ball.NONE) return;
        setHoodPosition(ball.isLarge() ? HOOD_LARGE : HOOD_SMALL);
    }

    /**
     * Reads the upcoming ball from Vision and adjusts the shooter mode and hood for it
     * @param vision - vision subsystem
     */
    public void setHoodFromVision(Vision vision) {
        if (vision == null) return;
        vision.updateBallSensor();
        setHoodForBall(vision.getNextBallForShooter());
    }

    /**
     * Reads the next ball detected by Vision heading into the shooter,
     * and sets the shooter mode (pollen vs nectar) accordingly.
     * @param vision - vision subsystem
     */
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
     * Runs the flywheel velocity control loop
     */
    public void update() {
        // velocity control here
    }

    /**
     * Safety stop function
     */
    public void stop() {
        spinning = false;
        motor.setPower(0);
    }
}