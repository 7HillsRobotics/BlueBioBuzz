package org.firstinspires.ftc.teamcode.opmodes.teleop;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sequences.*;

@SuppressWarnings("unused")
public abstract class MainTeleop extends LinearOpMode {

    protected Robot robot;
    protected Alliance alliance;
    protected DrivingSequences driveSequences;

    Action currentAction = null;

    public enum Alliance {
        RED, BLUE
    }

    double percent = 0.6;
    boolean fieldCentric  = true;
    boolean toggleControl = false;
    protected Pose2d targetPose = null;
    protected static final double STICK_DEADZONE = 0.05;
    protected abstract void setAlliance();
    protected abstract void setSequences();

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap);
        percent = 0.6;

        setAlliance();
        setSequences();

        // Initialize pose from "start" location if provided
        if (driveSequences != null) {
            Pose2d startPose = driveSequences.getLocations().get("start");
            if (startPose != null) {
                robot.setPose(startPose);
            }
        }

        waitForStart();

        while (opModeIsActive()) {
            // Driver 1 auto-score trigger
            if (gamepad1.a && currentAction == null) {
                currentAction = driveSequences.getActionTo("mainScorePos");
            }

            // Emergency stop
            if (gamepad2.left_bumper && gamepad2.right_bumper) {
                robot.stopAll();
                currentAction = null;
            }

            PoseVelocity2d velocity = robot.drivetrain.updatePose();

            actionLoop();

            if (currentAction == null) {
                driveLoop(velocity);
            }

            // Telemetry
            double heading = Math.toDegrees(robot.drivetrain.getPose().heading.toDouble());
            telemetry.addData("Drive Mode", fieldCentric ? "Field-Centric" : "Robot-Centric");
            telemetry.addData("Slow Mode",  percent < 1.0);
            telemetry.addData("Heading",    heading);
            telemetry.addData("Action Active", currentAction != null);

            if (targetPose != null) {
                Pose2d current = robot.drivetrain.getPose();
                double dx = targetPose.position.x - current.position.x;
                double dy = targetPose.position.y - current.position.y;
                telemetry.addData("Pose hold error (in)", String.format("x=%.2f  y=%.2f", dx, dy));
            }

        }

        robot.stopAll();
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

        // AI suggestion, needs to be tested
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

            // TODO check values
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
}