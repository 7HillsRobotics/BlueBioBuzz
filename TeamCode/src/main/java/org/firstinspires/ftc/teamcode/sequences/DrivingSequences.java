package org.firstinspires.ftc.teamcode.sequences;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@SuppressWarnings("unused")
public class DrivingSequences {

    private Drivetrain drivetrain;

    private final LinkedHashMap<String, Pose2d> locations = new LinkedHashMap<>();

    public DrivingSequences(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public void addLocation(String name, Pose2d pose) {
        locations.put(name, pose);
    }

    public Map<String, Pose2d> getLocations() {
        return locations;
    }

    public void setDrivetrain(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    /**
     * Drives all waypoints in insertion order starting from "start".
     * Each leg splines smoothly to the next waypoint.
     */
    public void runAll() {
        checkDrivetrain("runAll()");

        List<Map.Entry<String, Pose2d>> entries = new ArrayList<>(locations.entrySet());
        if (entries.size() < 2) return;

        Pose2d origin = entries.get(0).getValue();
        if (origin == null) return;

        TrajectoryActionBuilder builder = drivetrain.drive.actionBuilder(origin);
        boolean hasWaypoints = false;

        for (int i = 1; i < entries.size(); i++) {
            Pose2d target = entries.get(i).getValue();
            if (target == null) continue;
            builder = builder.splineTo(target.position, target.heading.toDouble());
            hasWaypoints = true;
        }

        if (hasWaypoints) {
            Actions.runBlocking(builder.build());
        }
    }

    /**
     * Returns an action that drives the robot from its current position to the named location.
     */
    public Action getActionTo(String name) {
        checkDrivetrain("getActionTo(\"" + name + "\")");
        Pose2d target = locations.get(name);
        if (target == null) {
            return null;
        }
        return drivetrain.drive.actionBuilder(drivetrain.getPose())
                .splineTo(target.position, target.heading.toDouble())
                .build();
    }

    public void runSequence(String name) {
        checkDrivetrain("runSequence(\"" + name + "\")");

        List<Map.Entry<String, Pose2d>> entries = new ArrayList<>(locations.entrySet());

        int targetIndex = -1;
        for (int i = 0; i < entries.size(); i++) {
            if (entries.get(i).getKey().equals(name)) {
                targetIndex = i;
                break;
            }
        }

        if (targetIndex < 0) {
            throw new IllegalArgumentException(
                    "DrivingSequences.runSequence(): no location named \"" + name + "\". " +
                            "Add it with addLocation() before calling runSequence().");
        }

        if (targetIndex == 0) return;

        Pose2d from   = entries.get(targetIndex - 1).getValue();
        Pose2d target = entries.get(targetIndex).getValue();
        if (from == null || target == null) return;

        Action action = drivetrain.drive.actionBuilder(from)
                .splineTo(target.position, target.heading.toDouble())
                .build();

        Actions.runBlocking(action);
    }

    private void checkDrivetrain(String caller) {
        if (drivetrain == null) {
            throw new IllegalStateException(
                    "DrivingSequences." + caller + " called but drivetrain is null. " +
                            "Call setDrivetrain() after Robot is constructed.");
        }
    }
}