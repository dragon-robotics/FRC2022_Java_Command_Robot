// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.General;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class VariableClimberCommand extends CommandBase {
  /** Creates a new VariableClimberCommand. */
  private final ClimberSubsystem m_climber;
  private final Supplier<Double> m_upSpeed;
  private final Supplier<Double> m_downSpeed;
  
  public VariableClimberCommand(
    ClimberSubsystem climber,
    Supplier<Double> upSpeed,
    Supplier<Double> downSpeed
  ) {
    m_climber = climber;
    m_upSpeed = upSpeed;
    m_downSpeed = downSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_upSpeed.get() > 0){
      m_climber.climb(m_upSpeed.get());
    }

    if(m_downSpeed.get() > 0){
      m_climber.climb(m_downSpeed.get());
    }
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
