package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sequences.DrivingSequences;
import org.firstinspires.ftc.teamcode.sequences.ScoringSequences;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/**
 * Abstract Base Class for Autonomous OpModes.
 * Handles drivetrain pathing, AprilTag turret alignment, position-based optimal shooter velocity,
 * and surgical tubing ball feeding.
 */
@SuppressWarnings("unused")
public abstract class MainAuto extends LinearOpMode {

    public enum Alliance { RED, BLUE }

    protected Alliance alliance;
    protected Robot robot;
    protected DrivingSequences driveSequences;
    protected ScoringSequences scoringSequences;

    protected abstract Alliance getDefaultAlliance();
    protected abstract Pose2d getStartPose();
    protected abstract void run();

    @Override
    public void runOpMode() {
        setup(getDefaultAlliance(), getStartPose());

        waitForStart();
        if (!opModeIsActive()) return;

        run();
    }

    protected void setup(Alliance alliance, Pose2d startPose) {
        this.alliance = alliance;
        this.driveSequences = new DrivingSequences(null);
        this.driveSequences.addLocation("start", startPose);

        this.robot = new Robot(hardwareMap, driveSequences);
        this.driveSequences.setDrivetrain(robot.drivetrain);
        this.scoringSequences = new ScoringSequences(robot, driveSequences);

        telemetry.addData("Status", "Initialized Autonomous: " + alliance);
        telemetry.addData("Start Pose", startPose);
        telemetry.update();
    }

    protected void driveToScore(Pose2d targetPose) {
        telemetry.addLine("Driving to target position...");
        telemetry.update();
        if (robot != null && robot.drivetrain != null) {
            Actions.runBlocking(
                    robot.drivetrain.drive.actionBuilder(robot.drivetrain.getPose())
                            .splineTo(targetPose.position, targetPose.heading.toDouble())
                            .build()
            );
        }
    }

    /**
     * Complete autonomous scoring sequence:
     * 1. Aims turret at AprilTags.
     * 2. Auto-calculates optimal flywheel RPM from current robot pose.
     * 3. Spins flywheel and waits for target velocity & alignment.
     * 4. Feeds ball via surgical tubing into shooter.
     * 5. Safely stops shooter and feeder.
     */
    protected void activateScoring() {
        telemetry.addLine("=== ACTIVATING SCORING SEQUENCE ===");
        telemetry.update();

        if (robot == null) return;

        Vision.AllianceColor visionAlliance = (alliance == Alliance.RED) ?
                Vision.AllianceColor.RED : Vision.AllianceColor.BLUE;

        // Step 1: Calculate optimal flywheel speed based on bot position & aim turret at AprilTags
        double targetRpm = 3000.0;
        if (robot.shooter != null) {
            targetRpm = robot.shooter.setTargetRpmFromPosition(robot.drivetrain.getPose());
            robot.shooter.setSpinning(true);
            if (robot.vision != null) {
                robot.shooter.setHoodFromVision(robot.vision);
            }
        }

        if (robot.turret != null) {
            robot.turret.setEnabled(true);
            robot.turret.pointToAprilTag(robot.vision);
        }

        // Step 2: Spin flywheel and run turret tracking until at speed and on target (with 2.0s timeout)
        long spinStartTime = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - spinStartTime < 2000)) {
            if (robot.shooter != null) robot.shooter.update();
            if (robot.turret != null) robot.turret.pointToAprilTag(robot.vision);

            boolean shooterReady = (robot.shooter == null) || robot.shooter.isAtSpeed();
            boolean turretReady = (robot.turret == null) || robot.turret.isOnTarget();

            telemetry.addData("Target Flywheel RPM", String.format("%.0f", targetRpm));
            if (robot.shooter != null) telemetry.addData("Actual Flywheel RPM", String.format("%.0f", robot.shooter.getRpm()));
            telemetry.addData("Shooter At Speed", shooterReady);
            telemetry.addData("Turret On Target", turretReady);
            telemetry.update();

            if (shooterReady && turretReady) {
                break;
            }
            sleep(20);
        }

        // Step 3: Feed ball into shooter flywheel using surgical tubing
        telemetry.addLine("Feeding ball via surgical tubing into shooter...");
        telemetry.update();

        if (robot.intake != null) {
            robot.intake.feedToShooter();
        }

        long feedStartTime = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - feedStartTime < 700)) {
            if (robot.shooter != null) robot.shooter.update();
            if (robot.turret != null) robot.turret.pointToAprilTag(robot.vision);
            sleep(20);
        }

        // Step 4: Stop intake feeder and shooter flywheel
        if (robot.intake != null) robot.intake.tubingOff();
        if (robot.shooter != null) robot.shooter.stop();

        telemetry.addLine("Scoring sequence complete!");
        telemetry.update();
    }

    protected void holdPose() {
        if (robot == null || robot.drivetrain == null) return;
        Pose2d target = robot.drivetrain.getPose();

        while (opModeIsActive()) {
            robot.drivetrain.updatePose();
            Pose2d current = robot.drivetrain.getPose();
            double heading = current.heading.toDouble();

            double worldDx = target.position.x - current.position.x;
            double worldDy = target.position.y - current.position.y;

            double robotDx = worldDx * Math.cos(heading) + worldDy * Math.sin(heading);
            double robotDy = -worldDx * Math.sin(heading) + worldDy * Math.cos(heading);

            double dh = target.heading.toDouble() - heading;
            while (dh > Math.PI) dh -= 2 * Math.PI;
            while (dh < -Math.PI) dh += 2 * Math.PI;

            robot.drivetrain.drive(robotDy * 0.12, robotDx * 0.12, dh * 1.1, 1.0);
        }
    }
}
