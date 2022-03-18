// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.GenerateTrajectory;
import frc.robot.AutoLoader.AutoCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.UptakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneBallTopLeftLowGoalCommand extends SequentialCommandGroup {
  /** Creates a new OneBallTopLeftLowGoalCommand. */

  private Trajectory p1, p2; // Part 1 and 2 of the trajectory

  // This command will require:
  // - Drivetrain subsystem
  // - Intake subsystem
  // - Uptake subsystem
  // - Shooter subsystem
  public OneBallTopLeftLowGoalCommand(
      DrivetrainSubsystem drivetrain,
      IntakeSubsystem intake,
      UptakeSubsystem uptake,
      ShooterSubsystem shooter,
      AutoCommand command) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // Get all the trajectories //
    List<Trajectory> trajectories = GenerateTrajectory.getTrajectory(command);

    p1 = trajectories.get(0);
    p2 = trajectories.get(1);

    /**
     * Command Sequence all ran in parallel:
     * 1. Move forward
     * 2. Shoot ball to low goal for 2 seconds
     * 3. Move backwards
     */
    addCommands(
        GenerateTrajectory.getRamseteCommand(p1, drivetrain),
        new ParallelRaceGroup(
            new WaitCommand(2), // shoot for 2 seconds
            new ShootAutoCommand(uptake, shooter)),
        GenerateTrajectory.getRamseteCommand(p2, drivetrain));
  }
}
