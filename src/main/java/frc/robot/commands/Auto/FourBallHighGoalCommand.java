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
public class FourBallHighGoalCommand extends ParallelCommandGroup {
  /** Creates a new FourBallHighGoalCommand. */

  private Trajectory p1, p2; // Part 1 and 2 of the trajectory

  // This command will require:
  // - Drivetrain subsystem
  // - Shooter subsystem
  // - Intake subsystem
  public FourBallHighGoalCommand(
      DrivetrainSubsystem drivetrain,
      AutoCommand command) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // Get all the trajectories //
    List<Trajectory> trajectories = GenerateTrajectory.getTrajectory(command);

    p1 = trajectories.get(0).concatenate(trajectories.get(1));
    p2 = trajectories.get(2).concatenate(trajectories.get(3));

    /**
     * Command Sequence all ran in parallel:
     * 1. Engage Shooter and Intake - <Parallel>
     * 2. new Sequential Command
     * a. Move backwards, intake ball, move forward
     * b. Shoot for x seconds
     * c. Move backwards, intake 2 more balls, move forward
     * d. Shoot for x seconds
     */
    addCommands(
        new SequentialCommandGroup(
            getRamseteCommand(p1, drivetrain),
            getRamseteCommand(p2, drivetrain)));
  }

  private Command getRamseteCommand(
      Trajectory trajectory,
      DrivetrainSubsystem drivetrain) {

    return new RamseteCommand(
        trajectory,
        drivetrain::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(
            Constants.ksVolts,
            Constants.kvVoltSecondsPerMeter,
            Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        drivetrain::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        drivetrain::tankDriveVolts,
        drivetrain)
            .beforeStarting(() -> drivetrain.resetOdometry(trajectory.getInitialPose()))
            .andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }
}
