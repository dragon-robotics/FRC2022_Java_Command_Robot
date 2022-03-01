// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.GenerateTrajectory;
import frc.robot.AutoLoader.AutoCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallTopLowGoalCommand extends ParallelCommandGroup {
  /** Creates a new TwoBallTopLowGoalCommand. */

  private Trajectory p1, p2; // Part 1 and 2 of the trajectory

  // This command will require:
  // - Drivetrain subsystem
  // - Shooter subsystem
  // - Intake subsystem
  public TwoBallTopLowGoalCommand(
    DrivetrainSubsystem drivetrain,
    AutoCommand command
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // Get all the trajectories //
    List<Trajectory> trajectories = GenerateTrajectory.getTrajectory(command);

    p1 = trajectories.get(0);
    p2 = trajectories.get(1);

    /**
     * Command Sequence all ran in parallel:
     * 1. Engage Shooter and Intake - <Parallel>
     * 2. new Sequential Command
     *  a. Move forward
     *  b. Shoot ball to low goal for x seconds
     *  c. Move backwards
     */
    addCommands(
      new SequentialCommandGroup(
        GenerateTrajectory.getRamseteCommand(p1, drivetrain),
        GenerateTrajectory.getRamseteCommand(p2, drivetrain)
      )
    );
  }
}
