package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@SuppressWarnings("unused")
@Autonomous(name = "Blue Auto")
public class BlueAuto extends MainAuto {

    private static final Pose2d START_POSE = new Pose2d(56,  8, Math.toRadians(90));
    private static final Pose2d SCORE_POSE = new Pose2d(-12,  34, Math.toRadians(270));
    private static final Pose2d PARK_POSE = new Pose2d(-12,  34, Math.toRadians(270));

    @Override
    protected Alliance getDefaultAlliance() { return Alliance.BLUE; }

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