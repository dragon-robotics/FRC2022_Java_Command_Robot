// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  CANSparkMax m_shooterL = new CANSparkMax(7, MotorType.kBrushless);
  CANSparkMax m_shooterR = new CANSparkMax(8, MotorType.kBrushless);

  public ShooterSubsystem() {
    // Inverting motor so motors spin in unison when connected to shooter
    m_shooterR.setInverted(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shoot() {
    m_shooterL.set(-0.1);
    m_shooterR.set(-0.1);

  }

  public void shoot(double speed){
    m_shooterL.set(-speed);
    m_shooterR.set(-speed);
  }

  public void stopMotor(){
    m_shooterL.set(0);
    m_shooterR.set(0);
  }
}
