// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  CANSparkMax m_shooter = new CANSparkMax(7, MotorType.kBrushless);

  public ShooterSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shootBackward() {
    m_shooter.set(0.85);
  }

  public void shootForward() {
    m_shooter.set(-0.85);

  }

  public void shoot(double speed){
    m_shooter.set(-speed);
  }

  public void stopMotor(){
    m_shooter.set(0);
  }
}
