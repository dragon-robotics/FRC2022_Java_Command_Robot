// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UptakeSubsystem extends SubsystemBase {
  /** Creates a new UptakeSubsystem. */
  public UptakeSubsystem() {}

  CANSparkMax m_uptakemotor1 = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax m_uptakemotor2 = new CANSparkMax(6, MotorType.kBrushless);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void motorUp() {
    m_uptakemotor1.set(0.5);
    m_uptakemotor2.set(0.5);
  }

  public void motorDown() {
    m_uptakemotor1.set(-0.5);
    m_uptakemotor2.set(-0.5);
  }
  
  public void motorOff() {
    m_uptakemotor1.set(0);
    m_uptakemotor2.set(0);
  }

  public void uptake(double speed) {
    m_uptakemotor1.set(speed);
    m_uptakemotor2.set(speed);
  }
}
