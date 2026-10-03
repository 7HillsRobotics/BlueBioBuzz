package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.sequences.DrivingSequences;

@SuppressWarnings("unused")
@TeleOp(name="Blue Teleop", group="Linear OpMode")
public class BlueTeleop extends MainTeleop {

    @Override
    protected void setAlliance() { alliance = Alliance.BLUE; }

    @Override
    protected void setSequences() {
        driveSequences = new DrivingSequences(robot.drivetrain);

        driveSequences.addLocation("start", new Pose2d(56,  8, Math.toRadians(90)));
        driveSequences.addLocation("mainScorePos", new Pose2d(0,  34, Math.toRadians(270)));
        // Add more teleop assist positions here as needed
    }
}