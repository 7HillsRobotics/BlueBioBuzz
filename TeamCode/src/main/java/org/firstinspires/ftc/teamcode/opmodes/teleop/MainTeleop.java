package org.firstinspires.ftc.teamcode.opmodes.teleop;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sequences.DrivingSequences;
import org.firstinspires.ftc.teamcode.sequences.ScoringSequences;

@SuppressWarnings("unused")
public abstract class MainTeleop extends LinearOpMode {

    protected Robot robot;
    protected Alliance alliance;
    protected DrivingSequences driveSequences;
    protected ScoringSequences scoringSequences;

    Action currentAction = null;

    public enum Alliance {
        RED, BLUE
    }

    double percent = 0.65;
    boolean fieldCentric  = true;
    boolean toggleControl = false;
    protected Pose2d targetPose = null;
    protected static final double STICK_DEADZONE = 0.05;

    // Subsystem States
    private boolean shooterSpinning = false;
    private boolean turretAutoTrack = true;

    // Button edge detection triggers
    private boolean g1APrev = false;
    private boolean g1BPrev = false;
    private boolean g1XPrev = false;
    private boolean g2APrev = false;
    private boolean g2XPrev = false;
    private boolean g2YPrev = false;
    private boolean g2DpadUpPrev = false;
    private boolean g2DpadDownPrev = false;

    protected abstract void setAlliance();
    protected abstract void setSequences();

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap);
        percent = 0.65;

        setAlliance();
        setSequences();

        if (robot != null) {
            scoringSequences = new ScoringSequences(robot, driveSequences);
        }

        // Initialize pose from "start" location if provided
        if (driveSequences != null && driveSequences.getLocations().containsKey("start")) {
            Pose2d startPose = driveSequences.getLocations().get("start");
            if (startPose != null) {
                robot.setPose(startPose);
            }
        }

        telemetry.addLine("TeleOp Initialized. Ready for START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- Vision Ball Sensor Update ---
            if (robot.vision != null) {
                robot.vision.updateBallSensor();
            }

            // --- Emergency Stop ---
            if ((gamepad1.left_bumper && gamepad1.right_bumper) ||
                (gamepad2.left_bumper && gamepad2.right_bumper) ||
                gamepad1.back || gamepad2.back) {
                robot.stopAll();
                currentAction = null;
                shooterSpinning = false;
                turretAutoTrack = false;
            }

            // --- Drivetrain Movement & Pose Update ---
            PoseVelocity2d velocity = robot.drivetrain.updatePose();

            actionLoop();

            if (currentAction == null) {
                driveLoop(velocity);
            }

            // --- Driver 1 Controls & Assist ---
            handleDriverControls();

            // --- Operator (Gamepad 2) Controls: Shooter, Turret, Intake ---
            handleOperatorControls();

            // --- Subsystem Updates ---
            if (robot.shooter != null) {
                robot.shooter.update();
            }

            // --- Telemetry Output ---
            updateTelemetry();
        }

        robot.stopAll();
    }

    /**
     * Driver 1 controls for drivetrain, auto-actions, position calculation, and quick auto-aim.
     */
    private void handleDriverControls() {
        // Driver 1 A: Auto-score RoadRunner trajectory action
        if (gamepad1.a && !g1APrev && currentAction == null && driveSequences != null) {
            if (driveSequences.getLocations().containsKey("mainScorePos")) {
                currentAction = driveSequences.getActionTo("mainScorePos");
            }
        }
        g1APrev = gamepad1.a;

        // Driver 1 B: Quick point turret to AprilTags
        if (gamepad1.b && !g1BPrev && robot.turret != null && robot.vision != null) {
            robot.turret.pointToAprilTag(robot.vision);
        }
        g1BPrev = gamepad1.b;

        // Driver 1 X: Quick calculate and set shooter target RPM from current bot position
        if (gamepad1.x && !g1XPrev && robot.shooter != null && robot.drivetrain != null) {
            robot.shooter.setTargetRpmFromPosition(robot.drivetrain.getPose());
        }
        g1XPrev = gamepad1.x;
    }

    /**
     * Operator (Gamepad 2) controls for Intake (rollers & surgical tubing), Shooter (flywheel speed), and Turret.
     */
    private void handleOperatorControls() {
        if (robot == null) return;

        // --- INTAKE CONTROLS ---
        if (robot.intake != null) {
            if (gamepad2.right_bumper) {
                // Outtake / reverse both intake rollers and surgical tubing
                robot.intake.outtake();
            } else {
                // Right Trigger: Front rollers suck in balls
                if (gamepad2.right_trigger > 0.1) {
                    robot.intake.setRollerPower(gamepad2.right_trigger);
                } else {
                    robot.intake.intakeOff();
                }

                // Left Trigger: Surgical tubing feeds ball to shooter
                if (gamepad2.left_trigger > 0.1) {
                    robot.intake.setTubingPower(gamepad2.left_trigger);
                } else {
                    robot.intake.tubingOff();
                }
            }
        }

        // --- SHOOTER FLYWHEEL CONTROLS ---
        if (robot.shooter != null) {
            // Gamepad 2 A: Toggle Shooter Flywheel Spinning
            if (gamepad2.a && !g2APrev) {
                shooterSpinning = !shooterSpinning;
                if (shooterSpinning) {
                    // Auto-calculate perfect speed from current bot pose if target RPM is 0
                    if (robot.drivetrain != null) {
                        robot.shooter.setTargetRpmFromPosition(robot.drivetrain.getPose());
                    } else if (robot.vision != null) {
                        robot.shooter.setTargetRpmFromVision(robot.vision);
                    }
                    if (robot.shooter.getTargetRpm() <= 0) {
                        robot.shooter.setTargetRpm(3000.0);
                    }
                }
                robot.shooter.setSpinning(shooterSpinning);
            }
            g2APrev = gamepad2.a;

            // Gamepad 2 D-Pad Up/Down: Adjust Target RPM (+/- 250 RPM)
            if (gamepad2.dpad_up && !g2DpadUpPrev) {
                robot.shooter.setTargetRpm(robot.shooter.getTargetRpm() + 250.0);
            }
            g2DpadUpPrev = gamepad2.dpad_up;

            if (gamepad2.dpad_down && !g2DpadDownPrev) {
                robot.shooter.setTargetRpm(robot.shooter.getTargetRpm() - 250.0);
            }
            g2DpadDownPrev = gamepad2.dpad_down;

            // Gamepad 2 Y: Auto update hood & mode from Vision ball color sensor
            if (gamepad2.y && !g2YPrev) {
                robot.shooter.updateModeFromVision(robot.vision);
            }
            g2YPrev = gamepad2.y;
        }

        // --- TURRET CONTROLS ---
        if (robot.turret != null) {
            // Gamepad 2 X: Toggle Auto AprilTag Tracking
            if (gamepad2.x && !g2XPrev) {
                turretAutoTrack = !turretAutoTrack;
                robot.turret.setEnabled(turretAutoTrack);
            }
            g2XPrev = gamepad2.x;

            double manualNudge = gamepad2.right_stick_x;
            if (turretAutoTrack && Math.abs(manualNudge) < 0.1) {
                // Auto AprilTag tracking
                robot.turret.pointToAprilTag(robot.vision);
            } else {
                // Manual operator nudge override
                robot.turret.manualNudge(manualNudge);
            }
        }
    }

    public void driveLoop(PoseVelocity2d velocity) {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rotInput = gamepad1.right_stick_x;

        // Field/robot-centric toggle
        if (gamepad1.y && !toggleControl) {
            fieldCentric = !fieldCentric;
            toggleControl = true;
        }
        if (!gamepad1.y) toggleControl = false;

        // Flip strafe for red alliance
        if (alliance == Alliance.RED) x *= -1;

        Pose2d current = robot.drivetrain.getPose();
        double heading = current.heading.toDouble();

        // Field centric conversion
        double robotX, robotY;
        if (fieldCentric) {
            robotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            robotY = x * Math.sin(-heading) + y * Math.cos(-heading);
        } else {
            robotX = x;
            robotY = y;
        }

        boolean driverMoving = Math.abs(robotX) > STICK_DEADZONE || Math.abs(robotY) > STICK_DEADZONE;
        boolean driverRotating = Math.abs(rotInput) > STICK_DEADZONE;
        boolean driverActive = driverMoving || driverRotating;

        if (driverActive || targetPose == null) {
            targetPose = current;
        }

        double cx = 0, cy = 0, cr = 0;

        if (!driverActive) {
            double worldDx = targetPose.position.x - current.position.x;
            double worldDy = targetPose.position.y - current.position.y;

            // rotate world-frame error into robot frame
            double robotDx =  worldDx * Math.cos(heading) + worldDy * Math.sin(heading);
            double robotDy = -worldDx * Math.sin(heading) + worldDy * Math.cos(heading);

            // wrap heading error to [-pi, pi]
            double dh = targetPose.heading.toDouble() - heading;
            while (dh >  Math.PI) dh -= 2 * Math.PI;
            while (dh < -Math.PI) dh += 2 * Math.PI;

            cx = robotDx * 0.12;
            cy = robotDy * 0.12;
            cr = dh * 1.1;
        }

        // Speed modes
        if (gamepad1.left_trigger  > 0.4) percent = 0.35;
        else if (gamepad1.right_trigger > 0.4) percent = 1.0;
        else percent = 0.65;

        // dpad micro adjustments
        if (gamepad1.dpad_up)    robotY =  0.18;
        if (gamepad1.dpad_down)  robotY = -0.18;
        if (gamepad1.dpad_left)  robotX = -0.18;
        if (gamepad1.dpad_right) robotX =  0.18;

        // Correction is not scaled by percent so it always has full authority
        robot.drivetrain.drive(
                robotY * percent + cy,
                robotX * percent + cx,
                rotInput * percent + cr,
                1.0
        );
    }

    public void actionLoop() {
        if (currentAction != null) {
            // Cancel action if driver tries to take control
            if (Math.abs(gamepad1.left_stick_x) > STICK_DEADZONE ||
                    Math.abs(gamepad1.left_stick_y) > STICK_DEADZONE ||
                    Math.abs(gamepad1.right_stick_x) > STICK_DEADZONE) {
                currentAction = null;
                return;
            }

            if (!currentAction.run(new TelemetryPacket())) {
                currentAction = null;
            }
        }
    }

    @SuppressLint("DefaultLocale")
    private void updateTelemetry() {
        Pose2d currentPose = robot.drivetrain.getPose();
        double headingDeg = Math.toDegrees(currentPose.heading.toDouble());

        telemetry.addLine("=== DRIVETRAIN & POSE ===");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Drive Mode", fieldCentric ? "Field-Centric" : "Robot-Centric");
        telemetry.addData("Speed Scaling", String.format("%.0f%%", percent * 100));
        telemetry.addData("Bot Position (in)", String.format("X: %.1f, Y: %.1f, Heading: %.1f°",
                currentPose.position.x, currentPose.position.y, headingDeg));

        telemetry.addLine("\n=== SHOOTER SUBSYSTEM ===");
        if (robot.shooter != null) {
            double calculatedOptimum = robot.shooter.calculatePerfectRpm(
                    Math.hypot(robot.shooter.getGoalPosition().x - currentPose.position.x,
                               robot.shooter.getGoalPosition().y - currentPose.position.y));
            telemetry.addData("Flywheel State (G2 A)", shooterSpinning ? "SPINNING" : "STOPPED");
            telemetry.addData("Target RPM (G2 DPad U/D)", String.format("%.0f RPM", robot.shooter.getTargetRpm()));
            telemetry.addData("Actual Measured RPM", String.format("%.0f RPM", robot.shooter.getRpm()));
            telemetry.addData("Calculated Optimum RPM", String.format("%.0f RPM", calculatedOptimum));
            telemetry.addData("Flywheel At Speed", robot.shooter.isAtSpeed() ? "READY TO SHOOT" : "SPINNING UP");
            telemetry.addData("Shooter Hood Mode (G2 Y)", robot.shooter.getMode());
        }

        telemetry.addLine("\n=== TURRET SUBSYSTEM ===");
        if (robot.turret != null) {
            telemetry.addData("Turret Tracking Mode (G2 X)", turretAutoTrack ? "AUTO APRILTAG TRACKING" : "MANUAL");
            telemetry.addData("Turret On Target", robot.turret.isOnTarget());
            telemetry.addData("Turret Bearing Error", String.format("%.1f°", robot.turret.getLastTurnErrorDeg()));
        }

        telemetry.addLine("\n=== INTAKE & VISION ===");
        if (robot.vision != null) {
            telemetry.addData("AprilTags Count", robot.vision.getTurretAprilTags().size());
            telemetry.addData("Next Ball Heading to Shooter", robot.vision.getNextBallForShooter());
        }

        telemetry.update();
    }
}
