package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sequences.DrivingSequences;
import org.firstinspires.ftc.teamcode.sequences.ScoringSequences;

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
        this.scoringSequences = new ScoringSequences(robot);

        telemetry.addData("Status", "Initialized: " + alliance);
        telemetry.addData("Start Pose", startPose);
        telemetry.update();
    }

    protected void driveToScore(Pose2d targetPose) {
        telemetry.addLine("Driving to score position...");
        telemetry.update();
        Actions.runBlocking(
                robot.drivetrain.drive.actionBuilder(robot.drivetrain.getPose())
                        .splineTo(targetPose.position, targetPose.heading.toDouble())
                        .build()
        );
    }

    protected void activateScoring() {
        telemetry.addLine("SCORING SEQUENCE ACTIVATED (Placeholder)");
        telemetry.update();
        if (scoringSequences != null) {
            scoringSequences.runScoringSequence();
        }
        sleep(1500);
    }

    protected void holdPose() {
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