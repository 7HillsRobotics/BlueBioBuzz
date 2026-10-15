package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem to control the intake mechanisms:
 * 1. Front rollers to draw/suck balls into the robot.
 * 2. Surgical tubing mechanism (might use rollers instead) to transport and feed balls up to the shooter.
 */
@SuppressWarnings("unused")
public class Intake {

    private final DcMotorEx rollerMotor;
    private final DcMotorEx tubingMotor;

    public static double DEFAULT_ROLLER_POWER = 0.8;
    public static double DEFAULT_TUBING_POWER = 0.8;

    public Intake(HardwareMap hw) {
        DcMotorEx tempRoller = null;
        try {
            tempRoller = hw.get(DcMotorEx.class, "intake");
        } catch (Exception e1) {
            try {
                tempRoller = hw.get(DcMotorEx.class, "rollers");
            } catch (Exception ignored) {}
        }
        this.rollerMotor = tempRoller;

        DcMotorEx tempTubing = null;
        try {
            tempTubing = hw.get(DcMotorEx.class, "tubing");
        } catch (Exception e1) {
            try {
                tempTubing = hw.get(DcMotorEx.class, "feeder");
            } catch (Exception ignored) {}
        }
        this.tubingMotor = tempTubing;
    }

    /**
     * Start intaking balls: runs front rollers and surgical tubing in forward direction.
     */
    public void intakeBalls() {
        // TODO: Run intake mechanisms to draw balls into the robot.
        // Suggestion: I would set both front rollers and surgical tubing motors to forward power.
    }

    /**
     * Turn front intake rollers ON.
     */
    public void intakeOn() {
        // TODO: Turn on front intake rollers.
        // Suggestion: I would set the front roller motor power to DEFAULT_ROLLER_POWER.
    }

    /**
     * Turn front intake rollers OFF.
     */
    public void intakeOff() {
        setRollerPower(0.0);
    }

    /**
     * Feed balls up to the shooter via surgical tubing.
     */
    public void feedToShooter() {
        // TODO: Transport balls up to the shooter.
        // Suggestion: I would set the surgical tubing feeder motor power to DEFAULT_TUBING_POWER.
    }

    /**
     * Stop surgical tubing feeder.
     */
    public void tubingOff() {
        setTubingPower(0.0);
    }

    /**
     * Reverses front rollers and surgical tubing to eject/outtake balls.
     */
    public void outtake() {
        // TODO: Eject balls from intake.
        // Suggestion: I would set negative power on both front rollers and surgical tubing motors to reverse direction.
    }

    /**
     * Set power of front intake rollers.
     * @param power - power level (-1.0 to 1.0)
     */
    public void setRollerPower(double power) {
        // TODO: Apply power to front intake rollers.
        // Suggestion: I would check if rollerMotor != null and call rollerMotor.setPower(power).
        if (rollerMotor != null) {
            rollerMotor.setPower(power);
        }
    }

    /**
     * Set power of surgical tubing mechanism.
     * @param power - power level (-1.0 to 1.0)
     */
    public void setTubingPower(double power) {
        // TODO: Apply power to surgical tubing feeder.
        // Suggestion: I would check if tubingMotor != null and call tubingMotor.setPower(power).
        if (tubingMotor != null) {
            tubingMotor.setPower(power);
        }
    }

    // --- Legacy / Compatibility Methods ---

    public void setPower(double power) {
        setRollerPower(power);
    }

    public void setFeederPower(double power) {
        setTubingPower(power);
    }

    public void feedToTurret() {
        feedToShooter();
    }

    public void feederOff() {
        tubingOff();
    }

    public void indexWindmill() {
        setTubingPower(0.5);
    }

    public void windmillOff() {
        tubingOff();
    }

    public void setWindmillPower(double power) {
        setTubingPower(power);
    }

    /**
     * Safety stop function for all intake hardware.
     */
    public void stop() {
        setRollerPower(0.0);
        setTubingPower(0.0);
    }
}
