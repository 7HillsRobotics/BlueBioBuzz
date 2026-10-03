package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@SuppressWarnings("unused")
@Autonomous(name = "Red Auto")
public class RedAuto extends MainAuto {

    private static final Pose2d START_POSE = new Pose2d(88, 8, Math.toRadians(90));
    private static final Pose2d SCORE_POSE = new Pose2d(-12, -34, Math.toRadians(90));
    private static final Pose2d PARK_POSE = new Pose2d(-12, -34, Math.toRadians(90));

    @Override
    protected Alliance getDefaultAlliance() { return Alliance.RED; }

    @Override
    protected Pose2d getStartPose() { return START_POSE; }

    @Override
    protected void run() {
        driveToScore(SCORE_POSE);
        activateScoring();
        driveToScore(PARK_POSE);
        holdPose();
    }
}