package org.firstinspires.ftc.teamcode.testFiles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/**
 * OpMode to test the Shooter subsystem (Flywheel motor, Hood servo, RPM/Mode switching).
 */
@SuppressWarnings("unused")
@TeleOp(name = "Test: Shooter", group = "Test")
public class TestShooter extends LinearOpMode {

    @Override
    public void runOpMode() {
        Shooter shooter = new Shooter(hardwareMap);
        Vision vision = new Vision(hardwareMap);

        telemetry.addLine("Shooter Test Ready. Press START.");
        telemetry.update();

        waitForStart();

        boolean spinning = false;
        double targetRpm = 2500.0;
        boolean aPrev = false;
        boolean xPrev = false;
        boolean yPrev = false;
        boolean dpadUpPrev = false;
        boolean dpadDownPrev = false;

        boolean isLargeHood = false;

        while (opModeIsActive()) {
            // Toggle Flywheel Spin
            if (gamepad1.a && !aPrev) {
                spinning = !spinning;
                shooter.setSpinning(spinning);
            }
            aPrev = gamepad1.a;

            // Adjust target RPM
            if (gamepad1.dpad_up && !dpadUpPrev) {
                targetRpm = Math.min(5000.0, targetRpm + 250.0);
                shooter.setTargetRpm(targetRpm);
            }
            dpadUpPrev = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !dpadDownPrev) {
                targetRpm = Math.max(0.0, targetRpm - 250.0);
                shooter.setTargetRpm(targetRpm);
            }
            dpadDownPrev = gamepad1.dpad_down;

            // Toggle Hood position manually
            if (gamepad1.x && !xPrev) {
                isLargeHood = !isLargeHood;
                shooter.setHoodPosition(isLargeHood ? Shooter.HOOD_LARGE : Shooter.HOOD_SMALL);
            }
            xPrev = gamepad1.x;

            // Auto mode update from Vision ball sensor
            if (gamepad1.y && !yPrev) {
                shooter.updateModeFromVision(vision);
            }
            yPrev = gamepad1.y;

            shooter.update();

            telemetry.addLine("=== SHOOTER SUBSYSTEM TEST ===");
            telemetry.addData("Flywheel State (A)", spinning ? "SPINNING" : "STOPPED");
            telemetry.addData("Target RPM (D-Pad U/D)", String.format("%.0f RPM", targetRpm));
            telemetry.addData("Hood Position (X)", String.format("%.2f (%s)",
                    shooter.getHoodPosition(), isLargeHood ? "NECTAR / LARGE" : "POLLEN / SMALL"));
            telemetry.addData("Current Mode", shooter.getMode());
            telemetry.addData("Next Ball from Vision (Y)", vision.getNextBallForShooter());
            telemetry.addLine("\nControls:");
            telemetry.addLine("A: Toggle Flywheel Spinning");
            telemetry.addLine("D-Pad Up/Down: Change Target RPM (+/- 250)");
            telemetry.addLine("X: Toggle Hood Position (Pollen/Small vs Nectar/Large)");
            telemetry.addLine("Y: Update Shooter Mode & Hood from Vision Color Sensor");
            telemetry.update();
        }

        shooter.stop();
        vision.close();
    }
}
