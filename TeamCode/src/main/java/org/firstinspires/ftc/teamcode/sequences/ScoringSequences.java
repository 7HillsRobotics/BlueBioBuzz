package org.firstinspires.ftc.teamcode.sequences;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.List;

/**
 * High-level sequences for collecting and scoring game elements.
 * Integrates Vision blob detection and DrivingSequences for element collection and scoring.
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
     * Collects pollen (yellow game element) using vision tracking and driving/intake.
     * @return true if a pollen blob was detected and tracked, false otherwise.
     */
    public boolean collectPollen() {
        if (intake != null) {
            intake.intakeOn();
            intake.indexWindmill();
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
     * Collects nectar (red or blue game element based on alliance) using vision tracking and driving/intake.
     * @param alliance Current alliance color
     * @return true if a nectar blob was detected and tracked, false otherwise.
     */
    public boolean collectNectar(Vision.AllianceColor alliance) {
        if (intake != null) {
            intake.intakeOn();
            intake.indexWindmill();
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
     * Prepares the shooter and hood for scoring.
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
                turret.update(vision);
            }
        }
    }

    /**
     * Feeds a ball into the shooter mechanism.
     */
    public void feedBall() {
        if (intake != null) {
            intake.feedToTurret();
            intake.indexWindmill();
        }
    }

    /**
     * Runs the full collection and scoring sequence sequentially.
     */
    public void runScoringSequence() {
        collectPollen();
        prepShooter(3000.0);
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
     * Action to collect pollen using vision-based driving and intake.
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
     * Action to collect nectar using vision-based driving and intake.
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
     * Action for spinning up the shooter.
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
                return shooter != null && !shooter.isAtSpeed();
            }
        };
    }

    /**
     * Action for aiming the turret.
     */
    public Action getAimTurretAction(Vision.AllianceColor alliance) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                packet.put("ScoringSequence", "Aiming turret");
                aimTurret(alliance);
                return turret != null && !turret.isOnTarget();
            }
        };
    }

    /**
     * Action for feeding a ball to score.
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
                packet.put("ScoringSequence", "Feeding ball");
                if (System.currentTimeMillis() - startTime > 500) {
                    if (intake != null) intake.feederOff();
                    return false;
                }
                return true;
            }
        };
    }

    /**
     * Action for full collection, aiming, and scoring sequence.
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
                        packet.put("ScoringSequence", "Step 0: Collecting game element (Vision & Drive)");
                        collectPollen();
                        stepStartTime = now;
                        step = 1;
                        break;
                    case 1:
                        packet.put("ScoringSequence", "Step 1: Intaking element...");
                        if (now - stepStartTime > 1500) {
                            if (robot != null && robot.drivetrain != null) robot.drivetrain.stop();
                            if (intake != null) intake.intakeOff();
                            step = 2;
                            stepStartTime = now;
                        } else {
                            collectPollen();
                        }
                        break;
                    case 2:
                        packet.put("ScoringSequence", "Step 2: Prepping shooter & mode...");
                        if (shooter != null) shooter.setHoodFromVision(vision);
                        prepShooter(3000.0);
                        step = 3;
                        stepStartTime = now;
                        break;
                    case 3:
                        packet.put("ScoringSequence", "Step 3: Aiming turret at AprilTags...");
                        aimTurret(alliance);
                        if (now - stepStartTime > 500) {
                            step = 4;
                            stepStartTime = now;
                        }
                        break;
                    case 4:
                        packet.put("ScoringSequence", "Step 4: Feeding ball & scoring...");
                        feedBall();
                        if (now - stepStartTime > 500) {
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
