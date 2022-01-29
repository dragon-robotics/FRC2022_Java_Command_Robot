// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsSubsystem extends SubsystemBase {
 
  DoubleSolenoid m_doublePCM1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
  DoubleSolenoid m_doublePCM2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);


  /** Creates a new PneumaticsSubsystem. */
  public PneumaticsSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void extend() {
    m_doublePCM1.set(kForward);
    m_doublePCM2.set(kForward);
  }

  public void retract() {
    m_doublePCM1.set(kReverse);
    m_doublePCM2.set(kReverse);
  }
}
