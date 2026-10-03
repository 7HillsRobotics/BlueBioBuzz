package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Class to control intake functionality: the intake roller, the windmill
 * that indexes balls inside the robot, and the feeder that sends a ball
 * up to the turret/shooter.
 */
public class Intake {

    // declare hardware here
    private final DcMotorEx motor;
    private final CRServo windmill;
    private final DcMotorEx feeder;

    public Intake(HardwareMap hw) {
        // initialize hardware here
        motor = hw.get(DcMotorEx.class, "intake");
        windmill = hw.get(CRServo.class, "windmill");
        feeder = hw.get(DcMotorEx.class, "feeder");
    }

    /**
     * Set the power of the intake roller
     * @param power - power to set
     */
    public void setPower(double power) {
        motor.setPower(power);
    }

    /**
     * Turn the intake roller on
     */
    public void intakeOn() {
        setPower(0.8);
    }

    /**
     * Turn the intake roller off
     */
    public void intakeOff() {
        setPower(0);
    }

    /**
     * Set the power of the internal windmill indexer
     * @param power - power to set
     */
    public void setWindmillPower(double power) {
        windmill.setPower(power);
    }

    /**
     * Index the windmill to the next ball position
     */
    public void indexWindmill() {
        setWindmillPower(0.5);
    }

    public void windmillOff() {
        setWindmillPower(0);
    }

    /**
     * Set the power of the feeder that sends a ball to the turret
     * @param power - power to set
     */
    public void setFeederPower(double power) {
        feeder.setPower(power);
    }

    /**
     * Feed a ball up to the turret/shooter
     */
    public void feedToTurret() {
        setFeederPower(0.8);
    }

    public void feederOff() {
        setFeederPower(0);
    }

    /**
     * Safety stop function
     */
    public void stop() {
        setPower(0);
        setWindmillPower(0);
        setFeederPower(0);
    }
}