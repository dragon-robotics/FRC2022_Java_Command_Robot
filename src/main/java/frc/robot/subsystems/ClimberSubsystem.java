// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new Climber. */

  CANSparkMax m_climberL = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax m_climberR = new CANSparkMax(8, MotorType.kBrushless);

  public ClimberSubsystem() {
    // Set climber to brake mode //
    m_climberL.setIdleMode(IdleMode.kBrake);
    m_climberR.setIdleMode(IdleMode.kBrake);
    
    // Invert 1 side of the climber //
    m_climberL.setInverted(true);
  }

  public void climb(double speed) {
    m_climberL.set(speed);
    m_climberR.set(speed);
  }

  public void stopMotor(){
    m_climberL.set(0);
    m_climberR.set(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
