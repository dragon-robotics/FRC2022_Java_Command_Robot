// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.GenerateTrajectory;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.AutoLoader.AutoCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallBotLowGoalCommand extends ParallelCommandGroup {
  /** Creates a new FiveBallBotLowGoalCommand. */

  private Trajectory p1, p2, p3; // Part 1, 2, and 3 of the trajectory

  // This command will require:
  // - Drivetrain subsystem
  // - Shooter subsystem
  // - Intake subsystem
  public FiveBallBotLowGoalCommand(
      DrivetrainSubsystem drivetrain,
      AutoCommand command
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // Get all the trajectories //
    List<Trajectory> trajectories = GenerateTrajectory.getTrajectory(command);

    p1 = trajectories.get(0).concatenate(trajectories.get(1));
    p2 = trajectories.get(2).concatenate(trajectories.get(3));
    p3 = trajectories.get(4).concatenate(trajectories.get(5));

    /**
     * Command Sequence all ran in parallel:
     * 1. Engage Shooter and Intake - <Parallel>
     * 2. new Sequential Command
     *  a. Move backwards, intake ball, move forward
     *  b. Shoot for x seconds
     *  c. Move backwards, intake 1 more ball, move forward
     *  d. Shoot for x seconds
     *  e. Move backwards, intake 2 more ball, move forward
     *  f. Shoot for x seconds
     */
    addCommands(
        new SequentialCommandGroup(
            GenerateTrajectory.getRamseteCommand(p1, drivetrain),
            GenerateTrajectory.getRamseteCommand(p2, drivetrain),
            GenerateTrajectory.getRamseteCommand(p3, drivetrain)));
  }
}
