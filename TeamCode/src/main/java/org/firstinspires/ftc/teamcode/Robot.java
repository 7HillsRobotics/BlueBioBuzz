package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.sequences.DrivingSequences;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

public class Robot {
    public Drivetrain drivetrain;
    public Intake intake;
    public Vision vision;
    public Turret turret;
    public Shooter shooter;
    private final GoBildaPinpointDriver pinpoint;

    public Robot(HardwareMap hw, DrivingSequences driveSequences) {
        pinpoint = hw.get(GoBildaPinpointDriver.class, "pinpoint");

        Pose2d startPose = new Pose2d(0, 0, 0);
        if (driveSequences != null && driveSequences.getLocations().containsKey("start")) {
            Pose2d found = driveSequences.getLocations().get("start");
            if (found != null) startPose = found;
        }

        setPose(startPose);

        drivetrain = new Drivetrain(hw, startPose);
        intake = new Intake(hw);
        vision = new Vision(hw);
        turret = new Turret(hw);
        shooter = new Shooter(hw);
    }

    public Robot(HardwareMap hw) {
        pinpoint = hw.get(GoBildaPinpointDriver.class, "pinpoint");

        drivetrain = new Drivetrain(hw, new Pose2d(0, 0, 0));
        intake = new Intake(hw);
        vision = new Vision(hw);
        turret = new Turret(hw);
        shooter = new Shooter(hw);
    }

    public void setPose(Pose2d pose) {
        if (pinpoint != null) {
            pinpoint.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
                    DistanceUnit.INCH, pose.position.x, pose.position.y,
                    AngleUnit.RADIANS, pose.heading.toDouble()));
        }
    }

    public void stopAll() {
        drivetrain.stop();
        intake.intakeOff();
        turret.stop();
        shooter.stop();
        vision.close();
    }
}