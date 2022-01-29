// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  CANSparkMax m_intakeL = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax m_intakeR = new CANSparkMax(1, MotorType.kBrushless);

  public IntakeSubsystem() {
    m_intakeR.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intake() {
    m_intakeL.set(0.1);
    m_intakeR.set(0.1);
  }

  public void intake(double speed) {
    m_intakeL.set(speed);
    m_intakeR.set(speed);
  }

  public void stopMotor() {
    m_intakeL.set(0);
    m_intakeR.set(0);
  }
}
