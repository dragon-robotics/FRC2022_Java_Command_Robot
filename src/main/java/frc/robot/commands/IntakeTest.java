// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeTest extends CommandBase {
  /** Creates a new IntakeTest. */

  private final IntakeSubsystem m_intake;     // Intake Subsystem
  private final Supplier<Double> m_speed;     // Speed
  private final Supplier<Boolean> m_extend;   // Extend Intake
  private final Supplier<Boolean> m_retract;  // Retract Intake

  public IntakeTest(
    IntakeSubsystem intake,
    Supplier<Double> speed,
    Supplier<Boolean> extend,
    Supplier<Boolean> retract
  ) {
    m_intake = intake;
    m_speed = speed;
    m_extend = extend;
    m_retract = retract;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Extend the intake //
    if(m_extend.get()){
      m_intake.pneumaticsExtend();
    }

    // Retract the intake //
    if (m_retract.get()) {
      m_intake.pneumaticsRetract();
    }

    // Vary intake motor speed //
    m_intake.variableMotor(m_speed.get());
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
