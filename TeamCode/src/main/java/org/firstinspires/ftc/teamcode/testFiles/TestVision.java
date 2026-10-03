package org.firstinspires.ftc.teamcode.testFiles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.List;

/**
 * OpMode to test Vision features: AprilTag detections, Pollen/Nectar color blob detection,
 * and the ball color sensor for upcoming shooter balls.
 */
@SuppressWarnings("unused")
@TeleOp(name = "Test: Vision", group = "Test")
public class TestVision extends LinearOpMode {

    @Override
    public void runOpMode() {
        Vision vision = new Vision(hardwareMap);

        telemetry.addLine("Vision Test Initialized. Press START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update ball color sensor
            vision.updateBallSensor();

            // AprilTag detections
            List<AprilTagDetection> turretTags = vision.getTurretAprilTags();
            List<AprilTagDetection> chassisTags = vision.getChassisAprilTags();

            // Color Blob detections
            List<ColorBlobLocatorProcessor.Blob> pollenBlobs = vision.getPollenBlobs();
            List<ColorBlobLocatorProcessor.Blob> redNectarBlobs = vision.getRedNectarBlobs();
            List<ColorBlobLocatorProcessor.Blob> blueNectarBlobs = vision.getBlueNectarBlobs();

            // Next ball & mode for shooter
            Vision.Ball nextBall = vision.getNextBallForShooter();
            Vision.ShooterMode recommendedMode = vision.getShooterModeForNextBall();

            telemetry.addLine("=== VISION SUBSYSTEM TEST ===");

            telemetry.addLine("\n--- Cameras Status ---");
            telemetry.addData("Turret Cam Ready", vision.isTurretCameraReady());
            telemetry.addData("Chassis Cam Ready", vision.isChassisCameraReady());

            telemetry.addLine("\n--- AprilTags ---");
            telemetry.addData("Turret Cam AprilTags Count", turretTags.size());
            for (AprilTagDetection tag : turretTags) {
                if (tag == null) continue;
                int tagId = (tag instanceof AprilTagSingleDetection) ? ((AprilTagSingleDetection) tag).id : -1;
                double bearing = (tag.ftcPose != null) ? tag.ftcPose.bearing : 0.0;
                double range = (tag.ftcPose != null) ? tag.ftcPose.range : 0.0;
                telemetry.addData("  Turret Tag ID " + tagId, String.format("Bearing: %.1f°, Range: %.1f in", bearing, range));
            }

            telemetry.addData("Chassis Cam AprilTags Count", chassisTags.size());
            for (AprilTagDetection tag : chassisTags) {
                if (tag == null) continue;
                int tagId = (tag instanceof AprilTagSingleDetection) ? ((AprilTagSingleDetection) tag).id : -1;
                double bearing = (tag.ftcPose != null) ? tag.ftcPose.bearing : 0.0;
                double range = (tag.ftcPose != null) ? tag.ftcPose.range : 0.0;
                telemetry.addData("  Chassis Tag ID " + tagId, String.format("Bearing: %.1f°, Range: %.1f in", bearing, range));
            }

            telemetry.addLine("\n--- Pollen & Nectar Color Blobs ---");
            telemetry.addData("Pollen (Yellow) Blobs Count", pollenBlobs.size());
            if (!pollenBlobs.isEmpty()) {
                ColorBlobLocatorProcessor.Blob top = pollenBlobs.get(0);
                telemetry.addData("  Top Pollen Center", String.format("X: %.0f, Y: %.0f, Area: %d",
                        top.getBoxFit().center.x, top.getBoxFit().center.y, top.getContourArea()));
            }

            telemetry.addData("Red Nectar Blobs Count", redNectarBlobs.size());
            telemetry.addData("Blue Nectar Blobs Count", blueNectarBlobs.size());

            telemetry.addLine("\n--- Ball Color Sensor (Shooter Queue) ---");
            telemetry.addData("Next Ball Heading to Shooter", nextBall);
            telemetry.addData("Recommended Shooter Mode", recommendedMode);

            telemetry.update();
            sleep(50);
        }

        vision.close();
    }
}
