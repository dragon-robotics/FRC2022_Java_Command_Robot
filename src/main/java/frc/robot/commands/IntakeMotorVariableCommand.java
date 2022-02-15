// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeMotorVariableCommand extends CommandBase {
  
  private final IntakeSubsystem m_intake;
  private final Supplier<Double> m_speed;
  
  /** Creates a new IntakeMotorVariableCommand. */
  public IntakeMotorVariableCommand(
    IntakeSubsystem intake,
    Supplier<Double> speed
  ) {
    m_intake = intake;
    m_speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.variableMotor(m_speed.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stops the motor when the command ends //
     m_intake.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
