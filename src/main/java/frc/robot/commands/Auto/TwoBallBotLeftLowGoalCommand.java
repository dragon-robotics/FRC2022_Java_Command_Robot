// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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
public class TwoBallBotLeftLowGoalCommand extends SequentialCommandGroup {
  /** Creates a new TwoBallBotLeftLowGoalCommand. */

  private Trajectory p1, p2; // Part 1 and 2 of the trajectory

  // This command will require:
  // - Drivetrain subsystem
  // - Intake subsystem
  // - Uptake subsystem
  // - Shooter subsystem
  public TwoBallBotLeftLowGoalCommand(
    DrivetrainSubsystem drivetrain,
    IntakeSubsystem intake,
    UptakeSubsystem uptake,
    ShooterSubsystem shooter,
    AutoCommand command
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // Get all the trajectories //
    List<Trajectory> trajectories = GenerateTrajectory.getTrajectory(command);

    p1 = trajectories.get(0);
    p2 = trajectories.get(1);

    /**
     * Command Sequence in sequential order:
     * 1. Intake cargo while moving backward
     * 2. Move forward while still intaking cargo for 2 more seconds
     * 3. Shoot ball for 2 seconds
     */
    addCommands(
        new ParallelDeadlineGroup(
            GenerateTrajectory.getRamseteCommand(p1, drivetrain),
            new IntakeCargoAutoCommand(intake, uptake)),
        new ParallelCommandGroup(
            GenerateTrajectory.getRamseteCommand(p2, drivetrain),
            new ParallelRaceGroup(
                // Intake the ball for 2 more seconds to make sure it's really in there
                new WaitCommand(2),
                new IntakeCargoAutoCommand(intake, uptake))),
        new ParallelRaceGroup(
            // Shoot the ball for 2 seconds
            new WaitCommand(2),
            new ShootAutoCommand(uptake, shooter)));
  }
}
