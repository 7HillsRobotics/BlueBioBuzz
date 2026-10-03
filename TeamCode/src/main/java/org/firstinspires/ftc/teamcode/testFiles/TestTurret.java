package org.firstinspires.ftc.teamcode.testFiles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * OpMode to test the Turret subsystem (CRServo, AprilTag tracking, manual nudge).
 */
@SuppressWarnings("unused")
@TeleOp(name = "Test: Turret", group = "Test")
public class TestTurret extends LinearOpMode {

    @Override
    public void runOpMode() {
        Turret turret = new Turret(hardwareMap);
        Vision vision = new Vision(hardwareMap);

        telemetry.addLine("Turret Test Ready. Press START.");
        telemetry.update();

        waitForStart();

        boolean autoTrack = true;
        boolean aPrev = false;
        boolean dpadUpPrev = false;
        boolean dpadDownPrev = false;
        int selectedTagId = -1;

        while (opModeIsActive()) {
            // Toggle Auto AprilTag tracking
            if (gamepad1.a && !aPrev) {
                autoTrack = !autoTrack;
                turret.setEnabled(autoTrack);
            }
            aPrev = gamepad1.a;

            // Cycle Target Tag ID (-1 for any, 1-10 for specific IDs)
            if (gamepad1.dpad_up && !dpadUpPrev) {
                selectedTagId++;
                turret.setTargetTagId(selectedTagId);
            }
            dpadUpPrev = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !dpadDownPrev) {
                if (selectedTagId > -1) selectedTagId--;
                turret.setTargetTagId(selectedTagId);
            }
            dpadDownPrev = gamepad1.dpad_down;

            double manualNudge = gamepad1.right_stick_x;

            if (autoTrack && Math.abs(manualNudge) < 0.1) {
                // Auto-track AprilTags using Vision
                turret.update(vision);
            } else {
                // Manual driver nudge override
                turret.manualNudge(manualNudge);
            }

            List<AprilTagDetection> detections = vision.getTurretAprilTags();

            telemetry.addLine("=== TURRET SUBSYSTEM TEST ===");
            telemetry.addData("Tracking Mode (A)", autoTrack ? "AUTO APRILTAG TRACKING" : "MANUAL");
            telemetry.addData("Target Tag ID (D-Pad U/D)", selectedTagId == -1 ? "ANY" : selectedTagId);
            telemetry.addData("Turret Is Tracking", turret.isTracking());
            telemetry.addData("Turret On Target", turret.isOnTarget());
            telemetry.addData("Turn Error (deg)", String.format("%.2f°", turret.getLastTurnErrorDeg()));
            telemetry.addData("Detected Tags Count", detections.size());
            telemetry.addData("Manual Stick Input", String.format("%.2f", manualNudge));
            telemetry.addLine("\nControls:");
            telemetry.addLine("A: Toggle Auto-Tracking");
            telemetry.addLine("D-Pad Up/Down: Change Target AprilTag ID");
            telemetry.addLine("Right Stick X: Manual Servo Nudge Override");
            telemetry.update();
        }

        turret.stop();
        vision.close();
    }
}
