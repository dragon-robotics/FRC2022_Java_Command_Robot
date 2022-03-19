// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TaxiCommand extends CommandBase {
  private final DrivetrainSubsystem m_drivetrain;
  private final double m_duration;
  private final double m_speed;
  private long m_startTime;

  
  /** Creates a new TaxiCommand. */
  public TaxiCommand(
    DrivetrainSubsystem drivetrain,
    double speed,
    double time
  ) {
    m_drivetrain = drivetrain;
    m_speed = speed;
    m_duration = time * 1000;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = System.currentTimeMillis();
    m_drivetrain.arcadeDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(m_speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    // Drive backwards for 2 seconds //
    return (System.currentTimeMillis() - m_startTime) >= m_duration;
  }
}
