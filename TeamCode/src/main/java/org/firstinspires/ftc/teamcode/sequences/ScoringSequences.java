package org.firstinspires.ftc.teamcode.sequences;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.List;

/**
 * High-level sequences for collecting and scoring game elements.
 * Integrates Vision blob detection, Shooter speed calculation based on bot position/vision,
 * Turret AprilTag tracking, and Intake front rollers/surgical tubing transport.
 */
@SuppressWarnings("unused")
public class ScoringSequences {

    private Robot robot;
    private Intake intake;
    private Turret turret;
    private Shooter shooter;
    private Vision vision;
    private DrivingSequences driveSequences;

    public ScoringSequences(Robot robot) {
        setRobot(robot);
    }

    public ScoringSequences(Robot robot, DrivingSequences driveSequences) {
        setRobot(robot);
        this.driveSequences = driveSequences;
    }

    public ScoringSequences(Intake intake, Turret turret, Shooter shooter, Vision vision) {
        this.intake = intake;
        this.turret = turret;
        this.shooter = shooter;
        this.vision = vision;
    }

    public void setRobot(Robot robot) {
        this.robot = robot;
        if (robot != null) {
            this.intake = robot.intake;
            this.turret = robot.turret;
            this.shooter = robot.shooter;
            this.vision = robot.vision;
        }
    }

    public Robot getRobot() {
        return robot;
    }

    public void setDrivingSequences(DrivingSequences driveSequences) {
        this.driveSequences = driveSequences;
    }

    public DrivingSequences getDrivingSequences() {
        return driveSequences;
    }

    /**
     * Collects pollen (yellow game element) using front intake rollers and vision tracking.
     * @return true if a pollen blob was detected and tracked, false otherwise.
     */
    public boolean collectPollen() {
        if (intake != null) {
            intake.intakeOn();
            intake.feedToShooter();
        }

        if (vision != null && robot != null && robot.drivetrain != null) {
            List<ColorBlobLocatorProcessor.Blob> pollenBlobs = vision.getPollenBlobs();
            if (!pollenBlobs.isEmpty()) {
                ColorBlobLocatorProcessor.Blob bestBlob = pollenBlobs.get(0);
                for (ColorBlobLocatorProcessor.Blob blob : pollenBlobs) {
                    if (blob.getContourArea() > bestBlob.getContourArea()) {
                        bestBlob = blob;
                    }
                }

                double centerX = bestBlob.getBoxFit().center.x;
                double xErr = (centerX - 320.0) / 320.0;

                double vx = 0.30;
                double omega = xErr * 0.40;
                robot.drivetrain.drive(vx, 0.0, omega, 1.0);
                return true;
            }
        }

        if (driveSequences != null && driveSequences.getLocations().containsKey("pollenPos")) {
            driveSequences.runSequence("pollenPos");
            return true;
        }

        return false;
    }

    /**
     * Collects nectar (red or blue game element based on alliance) using intake rollers and vision.
     * @param alliance Current alliance color
     * @return true if a nectar blob was detected and tracked, false otherwise.
     */
    public boolean collectNectar(Vision.AllianceColor alliance) {
        if (intake != null) {
            intake.intakeOn();
            intake.feedToShooter();
        }

        if (vision != null && robot != null && robot.drivetrain != null) {
            List<ColorBlobLocatorProcessor.Blob> nectarBlobs = vision.getNectarBlobs(alliance);
            if (!nectarBlobs.isEmpty()) {
                ColorBlobLocatorProcessor.Blob bestBlob = nectarBlobs.get(0);
                for (ColorBlobLocatorProcessor.Blob blob : nectarBlobs) {
                    if (blob.getContourArea() > bestBlob.getContourArea()) {
                        bestBlob = blob;
                    }
                }

                double centerX = bestBlob.getBoxFit().center.x;
                double xErr = (centerX - 320.0) / 320.0;

                double vx = 0.30;
                double omega = xErr * 0.40;
                robot.drivetrain.drive(vx, 0.0, omega, 1.0);
                return true;
            }
        }

        String locName = (alliance == Vision.AllianceColor.RED) ? "redNectarPos" : "blueNectarPos";
        if (driveSequences != null && driveSequences.getLocations().containsKey(locName)) {
            driveSequences.runSequence(locName);
            return true;
        }

        return false;
    }

    /**
     * Automatically calculates the ideal flywheel speed based on the robot's current position
     * (or vision AprilTag range) and spins up the shooter.
     *
     * @return calculated target RPM
     */
    public double prepShooterAuto() {
        if (shooter == null) return 0.0;

        double calculatedRpm;
        if (robot != null && robot.drivetrain != null) {
            Pose2d currentPose = robot.drivetrain.getPose();
            calculatedRpm = shooter.setTargetRpmFromPosition(currentPose);
        } else if (vision != null) {
            calculatedRpm = shooter.setTargetRpmFromVision(vision);
        } else {
            calculatedRpm = 3000.0;
            shooter.setTargetRpm(calculatedRpm);
        }

        shooter.setSpinning(true);
        if (vision != null) {
            shooter.setHoodFromVision(vision);
        }
        return calculatedRpm;
    }

    /**
     * Prepares the shooter and hood for scoring with a manually specified RPM.
     *
     * @param targetRpm Target flywheel RPM
     */
    public void prepShooter(double targetRpm) {
        if (shooter != null) {
            shooter.setTargetRpm(targetRpm);
            shooter.setSpinning(true);
            if (vision != null) {
                shooter.setHoodFromVision(vision);
            }
        }
    }

    /**
     * Aims the turret toward detected AprilTags using vision.
     *
     * @param alliance Current alliance color
     */
    public void aimTurret(Vision.AllianceColor alliance) {
        if (turret != null) {
            turret.setEnabled(true);
            if (vision != null) {
                turret.pointToAprilTag(vision);
            }
        }
    }

    /**
     * Feeds a ball into the shooter using the surgical tubing mechanism.
     */
    public void feedBall() {
        if (intake != null) {
            intake.feedToShooter();
        }
    }

    /**
     * Runs the full collection, auto-speed calculation, aiming, and scoring sequence.
     */
    public void runScoringSequence() {
        prepShooterAuto();
        aimTurret(Vision.AllianceColor.RED);
        feedBall();
    }

    /**
     * Stops all scoring and intake subsystems.
     */
    public void stopScoring() {
        if (shooter != null) shooter.stop();
        if (turret != null) turret.stop();
        if (intake != null) intake.stop();
        if (robot != null && robot.drivetrain != null) robot.drivetrain.stop();
    }

    // --- Road Runner Actions ---

    /**
     * Action to collect pollen using vision-based driving and intake rollers.
     */
    public Action getCollectPollenAction() {
        return new Action() {
            private long startTime = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (startTime == 0) startTime = System.currentTimeMillis();
                long elapsed = System.currentTimeMillis() - startTime;

                packet.put("ScoringSequence", "Collecting Pollen (Vision & Drive)");
                collectPollen();

                if (elapsed > 2000) {
                    if (robot != null && robot.drivetrain != null) robot.drivetrain.stop();
                    if (intake != null) intake.intakeOff();
                    return false;
                }
                return true;
            }
        };
    }

    /**
     * Action to collect nectar using vision-based driving and intake rollers.
     */
    public Action getCollectNectarAction(Vision.AllianceColor alliance) {
        return new Action() {
            private long startTime = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (startTime == 0) startTime = System.currentTimeMillis();
                long elapsed = System.currentTimeMillis() - startTime;

                packet.put("ScoringSequence", "Collecting Nectar (Vision & Drive)");
                collectNectar(alliance);

                if (elapsed > 2000) {
                    if (robot != null && robot.drivetrain != null) robot.drivetrain.stop();
                    if (intake != null) intake.intakeOff();
                    return false;
                }
                return true;
            }
        };
    }

    /**
     * Action for spinning up the shooter to optimal calculated RPM based on position/vision.
     */
    public Action getPrepShooterAutoAction() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    double rpm = prepShooterAuto();
                    packet.put("ScoringSequence", "Calculated Optimal RPM: " + String.format("%.0f", rpm));
                    initialized = true;
                }

                if (shooter != null) {
                    shooter.update();
                    packet.put("Shooter Target RPM", shooter.getTargetRpm());
                    packet.put("Shooter Actual RPM", shooter.getRpm());
                    return !shooter.isAtSpeed();
                }
                return false;
            }
        };
    }

    /**
     * Action for spinning up the shooter to a target RPM.
     */
    public Action getPrepShooterAction(double targetRpm) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    packet.put("ScoringSequence", "Spinning up shooter to " + targetRpm + " RPM");
                    prepShooter(targetRpm);
                    initialized = true;
                }

                if (shooter != null) {
                    shooter.update();
                    return !shooter.isAtSpeed();
                }
                return false;
            }
        };
    }

    /**
     * Action for aiming the turret toward AprilTags.
     */
    public Action getAimTurretAction(Vision.AllianceColor alliance) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                packet.put("ScoringSequence", "Aiming turret at AprilTags");
                aimTurret(alliance);
                return turret != null && !turret.isOnTarget();
            }
        };
    }

    /**
     * Action for feeding a ball into the shooter via surgical tubing.
     */
    public Action getFeedBallAction() {
        return new Action() {
            private long startTime = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (startTime == 0) {
                    startTime = System.currentTimeMillis();
                    feedBall();
                }
                packet.put("ScoringSequence", "Feeding ball via surgical tubing");
                if (System.currentTimeMillis() - startTime > 600) {
                    if (intake != null) intake.tubingOff();
                    return false;
                }
                return true;
            }
        };
    }

    /**
     * Action for full aiming, shooter spinup to perfect RPM, and feeding sequence.
     */
    public Action getCollectAndScoreAction(Vision.AllianceColor alliance) {
        return new Action() {
            private int step = 0;
            private long stepStartTime = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                long now = System.currentTimeMillis();

                switch (step) {
                    case 0:
                        packet.put("ScoringSequence", "Step 0: Calculating perfect RPM & Aiming turret...");
                        prepShooterAuto();
                        aimTurret(alliance);
                        stepStartTime = now;
                        step = 1;
                        break;
                    case 1:
                        if (shooter != null) shooter.update();
                        if (turret != null) turret.pointToAprilTag(vision);

                        packet.put("ScoringSequence", "Step 1: Waiting for flywheel & turret alignment...");
                        boolean shooterReady = (shooter == null) || shooter.isAtSpeed();
                        boolean turretReady = (turret == null) || turret.isOnTarget();

                        if ((shooterReady && turretReady) || (now - stepStartTime > 2000)) {
                            step = 2;
                            stepStartTime = now;
                        }
                        break;
                    case 2:
                        packet.put("ScoringSequence", "Step 2: Feeding ball via surgical tubing into shooter...");
                        feedBall();
                        if (now - stepStartTime > 600) {
                            stopScoring();
                            return false; // Action complete
                        }
                        break;
                }
                return true;
            }
        };
    }

    /**
     * Action placeholder for complete scoring sequence.
     */
    public Action getScoreAction() {
        return getCollectAndScoreAction(Vision.AllianceColor.RED);
    }
}
