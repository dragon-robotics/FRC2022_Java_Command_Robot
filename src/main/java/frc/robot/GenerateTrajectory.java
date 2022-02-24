// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.AutoLoader.AutoCommand;

/** Generates Trajectories depending on which auto mode is selected */
public class GenerateTrajectory {

    public static Map<String, List<Trajectory>> trajMap = new HashMap<>();

    public static void loadTrajectories() {

        // Get all JSON files from the deploy directory //

        File[] trajJsonFiles = Filesystem.getDeployDirectory().listFiles(
                (d, s) -> {
                    return s.toLowerCase().endsWith(".json");
                });

        try {
            // For each trajectory JSON files, parse the commands from the filename
            // and group the paths into { command => trajectory[] } pairs.
            for (File trajJson : trajJsonFiles) {

                String filename = trajJson.getName();
                System.out.println(trajJson.getName()); // Print name for debug

                // Get path //
                Path trajPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
                Trajectory traj = TrajectoryUtil.fromPathweaverJson(trajPath);

                // Parse command //
                String command = filename.split("-", 2)[0]; // Command is always 1st half
                // Load trajectory //

                // Check if command exist in hashmap //
                if (trajMap.containsKey(command)) {
                    // If command exist, insert the trajectory into the existing array //
                    trajMap.get(command).add(traj);
                } else {
                    // If command doesn't exist, insert a new command + trajectory array //
                    List<Trajectory> trajList = new ArrayList<Trajectory>(
                            Arrays.asList(traj));
                    trajMap.put(command, trajList);
                }
            }
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory", ex.getStackTrace());
        }
    }

    public static List<Trajectory> getTrajectory(AutoCommand command) {
        switch (command) {
            case NONE:
                return null;
            case EXAMPLE_TRAJECTORY:

                // Create a voltage constraint to ensure we don't accelerate too fast
                DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(
                                Constants.ksVolts,
                                Constants.kvVoltSecondsPerMeter,
                                Constants.kaVoltSecondsSquaredPerMeter),
                        Constants.kDriveKinematics,
                        10);

                // Generic Trajectory Config Information //
                TrajectoryConfig config = new TrajectoryConfig(
                        Constants.kMaxSpeedMetersPerSecond,
                        Constants.kMaxAccelerationMetersPerSecondSquared)
                                // Add kinematics to ensure max speed is actually obeyed
                                .setKinematics(Constants.kDriveKinematics)
                                // Apply the voltage constraint
                                .addConstraint(autoVoltageConstraint);

                Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(
                                new Translation2d(1, 2),
                                new Translation2d(3, 1),
                                new Translation2d(2, 0),
                                new Translation2d(3, -1),
                                new Translation2d(1, -2)),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(0, 0, new Rotation2d(Math.toRadians(-180))),
                        // Pass config
                        config);

                return new ArrayList<Trajectory>(Arrays.asList(exampleTrajectory));
            case FOUR_BALL_BOT_LEFT_HIGH_GOAL:
                return trajMap.get("FourBallBotLeftHighGoal");
            default:
                return null;
        }
    }
}
