// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.General;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.UptakeSubsystem;

public class VariableUptakeCommand extends CommandBase {
  /** Creates a new VariableUptakeCommand. */

  private final UptakeSubsystem m_uptake;
  private final Supplier<Double> m_speed;

  public VariableUptakeCommand(
    UptakeSubsystem uptake,
    Supplier<Double> speed
  ) {

    m_uptake = uptake;
    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(uptake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = m_speed.get();
    m_uptake.uptake(speed);
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
