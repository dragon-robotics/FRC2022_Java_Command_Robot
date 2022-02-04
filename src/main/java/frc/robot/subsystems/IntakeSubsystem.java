// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  CANSparkMax m_intakeL = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax m_intakeR = new CANSparkMax(1, MotorType.kBrushless);

  DoubleSolenoid m_doublePCM6 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
  DoubleSolenoid m_doublePCM7 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

  Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  public IntakeSubsystem() {
    m_intakeR.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Intake motor commands
  public void engageMotor() {
    m_intakeL.set(0.1);
    m_intakeR.set(0.1);
  }

  public void stopMotor() {
    m_intakeL.set(0);
    m_intakeR.set(0);
  }

  // Intake pneumatics commands
  public void pneumaticsExtend() {
    m_doublePCM6.set(kForward);
    m_doublePCM7.set(kForward);
  }

  public void pneumaticsRetract() {
    m_doublePCM6.set(kOff);
    m_doublePCM7.set(kOff);
  }

  public void compressorRun(){
    pcmCompressor.enableDigital();
  }

  public void compressorEnd() {
    pcmCompressor.disable();
  }
}
