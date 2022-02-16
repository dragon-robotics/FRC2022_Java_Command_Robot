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

<<<<<<< HEAD
  // CANSparkMax m_intakeL = new CANSparkMax(1, MotorType.kBrushless);
  // CANSparkMax m_intakeR = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax m_intake = new CANSparkMax(4, MotorType.kBrushless);
=======
  CANSparkMax m_intakeMotor = new CANSparkMax(1, MotorType.kBrushless);
>>>>>>> fd4e64b74b1b573d9830206816e21aefdcbabd09

  DoubleSolenoid m_doublePCM1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
  DoubleSolenoid m_doublePCM2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);

  Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

<<<<<<< HEAD
  public IntakeSubsystem() {
    // m_intakeR.setInverted(true);
  }
=======
  public IntakeSubsystem() {}
>>>>>>> fd4e64b74b1b573d9830206816e21aefdcbabd09

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void variedMotor(double motorSpeed) {
    m_intake.set(motorSpeed);
  }

  // Intake motor commands
  public void engageMotor() {
<<<<<<< HEAD
    // m_intakeL.set(0.1);
    // m_intakeR.set(0.1);

    m_intake.set(0.1);

  }

  public void stopMotor() {
    // m_intakeL.set(0);
    // m_intakeR.set(0);

    m_intake.set(0);
=======
    m_intakeMotor.set(0.1);
  }

  public void stopMotor() {
    m_intakeMotor.set(0);
  }

  public void variableMotor(double speed){
    m_intakeMotor.set(speed);
>>>>>>> fd4e64b74b1b573d9830206816e21aefdcbabd09
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

  public void compressorEnd() {
    pcmCompressor.disable();
  }
}
