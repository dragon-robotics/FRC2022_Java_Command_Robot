// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ArcadeDriveCommand extends CommandBase {

  private final DrivetrainSubsystem m_drivetrain;
  private final Supplier<Double> m_speed;
  private final Supplier<Double> m_rotation;

  /** Creates a new ArcadeDrive. */
  public ArcadeDriveCommand(
    DrivetrainSubsystem drivetrain,
    Supplier<Double> speed,
    Supplier<Double> rotation) {
    m_drivetrain = drivetrain;
    m_speed = speed;
    m_rotation = rotation;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(m_speed.get(), m_rotation.get());
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
