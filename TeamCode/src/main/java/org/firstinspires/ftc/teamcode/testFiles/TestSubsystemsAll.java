package org.firstinspires.ftc.teamcode.testFiles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Integrated OpMode to test all robot subsystems (Drivetrain, Intake, Turret, Shooter, Vision) together.
 */
@SuppressWarnings("unused")
@TeleOp(name = "Test: All Subsystems", group = "Test")
public class TestSubsystemsAll extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);

        telemetry.addLine("All Subsystems Test Initialized. Press START.");
        telemetry.update();

        waitForStart();

        boolean autoTurret = true;
        boolean shooterSpinning = false;
        boolean aPrev = false;
        boolean bPrev = false;
        boolean xPrev = false;

        while (opModeIsActive()) {
            // Update Vision ball color sensor
            if (robot.vision != null) {
                robot.vision.updateBallSensor();
            }

            // Gamepad 1: Drivetrain Mecanum Controls
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            double speedPercent = gamepad1.left_bumper ? 0.35 : 0.70;

            if (robot.drivetrain != null) {
                robot.drivetrain.drive(drive, strafe, rotate, speedPercent);
            }

            // Gamepad 1 A: Toggle Auto AprilTag Turret Tracking
            if (gamepad1.a && !aPrev) {
                autoTurret = !autoTurret;
                if (robot.turret != null) robot.turret.setEnabled(autoTurret);
            }
            aPrev = gamepad1.a;

            // Turret Tracking
            if (robot.turret != null) {
                if (autoTurret) {
                    robot.turret.update(robot.vision);
                } else {
                    robot.turret.manualNudge(gamepad1.right_trigger - gamepad1.left_trigger);
                }
            }

            // Gamepad 2: Intake Controls
            if (robot.intake != null) {
                if (gamepad2.right_trigger > 0.1) {
                    robot.intake.intakeOn();
                    robot.intake.indexWindmill();
                } else {
                    robot.intake.intakeOff();
                    robot.intake.windmillOff();
                }

                if (gamepad2.left_trigger > 0.1) {
                    robot.intake.feedToTurret();
                } else {
                    robot.intake.feederOff();
                }
            }

            // Gamepad 2 A: Toggle Shooter Flywheel
            if (gamepad2.a && !bPrev) {
                shooterSpinning = !shooterSpinning;
                if (robot.shooter != null) {
                    robot.shooter.setTargetRpm(3000.0);
                    robot.shooter.setSpinning(shooterSpinning);
                }
            }
            bPrev = gamepad2.a;

            // Gamepad 2 X: Update Shooter Hood & Mode from Vision
            if (gamepad2.x && !xPrev) {
                if (robot.shooter != null) {
                    robot.shooter.updateModeFromVision(robot.vision);
                }
            }
            xPrev = gamepad2.x;

            // Emergency Stop
            if (gamepad1.back || gamepad2.back) {
                robot.stopAll();
                autoTurret = false;
                shooterSpinning = false;
            }

            telemetry.addLine("=== ALL SUBSYSTEMS INTEGRATED TEST ===");
            telemetry.addData("Drivetrain Mode", speedPercent < 0.5 ? "SLOW" : "NORMAL");
            telemetry.addData("Turret Auto-Track (G1 A)", autoTurret ? "ON" : "OFF");
            if (robot.turret != null) {
                telemetry.addData("Turret On Target", robot.turret.isOnTarget());
                telemetry.addData("Turret Error", String.format("%.1f°", robot.turret.getLastTurnErrorDeg()));
            }
            telemetry.addData("Shooter Flywheel (G2 A)", shooterSpinning ? "SPINNING (3000 RPM)" : "STOPPED");
            if (robot.shooter != null) {
                telemetry.addData("Shooter Mode", robot.shooter.getMode());
            }
            if (robot.vision != null) {
                telemetry.addData("Next Ball Heading to Shooter", robot.vision.getNextBallForShooter());
                telemetry.addData("AprilTags Count", robot.vision.getTurretAprilTags().size());
                telemetry.addData("Pollen Blobs Count", robot.vision.getPollenBlobs().size());
            }

            telemetry.update();
        }

        robot.stopAll();
    }
}
