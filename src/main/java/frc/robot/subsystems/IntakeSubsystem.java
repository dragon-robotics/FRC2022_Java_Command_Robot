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

  CANSparkMax m_intakeMotor = new CANSparkMax(2, MotorType.kBrushless);

  DoubleSolenoid m_doublePCM1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  DoubleSolenoid m_doublePCM2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

  Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Intake motor commands
  public void forwardMotor() {
    m_intakeMotor.set(0.5);
  }

  public void reverseMotor() {
    m_intakeMotor.set(-0.5);
  }

  public void stopMotor() {
    m_intakeMotor.set(0);
  }

  public void variableMotor(double speed){
    m_intakeMotor.set(speed);
  }

  // Intake pneumatics commands
  public void pneumaticsExtend() {
    m_doublePCM1.set(kForward);
    m_doublePCM2.set(kForward);

  }

  public void pneumaticsRetract() {
    m_doublePCM1.set(kReverse);
    m_doublePCM2.set(kReverse);

  }

  public void pneumaticsNeutral() {
    m_doublePCM1.set(kOff);
    m_doublePCM2.set(kOff);

  }

  public void compressorOff() {
    pcmCompressor.disable();
  }
  public void compressorOn(){
    pcmCompressor.enableDigital();
  }
}
