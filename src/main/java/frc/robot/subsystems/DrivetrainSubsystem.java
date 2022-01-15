// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;

public class DrivetrainSubsystem extends SubsystemBase {
  
  
  // Declare subsystem attribute/components //
  
  // Motor Controllers //
  WPI_TalonFX m_talonLT = new WPI_TalonFX(2);
  WPI_TalonFX m_talonLB = new WPI_TalonFX(1);
  WPI_TalonFX m_talonRT = new WPI_TalonFX(4);
  WPI_TalonFX m_talonRB = new WPI_TalonFX(3);
  MotorControllerGroup m_motorL = new MotorControllerGroup(m_talonLB, m_talonLT);
  MotorControllerGroup m_motorR = new MotorControllerGroup(m_talonRB, m_talonRT);
  DifferentialDrive m_drive = new DifferentialDrive(m_motorL, m_motorR);

  // Encoders //
  
  // NavX //
  AHRS ahrs = new AHRS(SPI.Port.kMXP); 

  /** Creates a new Drivetrain. */
  public DrivetrainSubsystem() {

    // Create TalonFX Motor Controller for motor controls //

    // Initialize TalonFX to be 0 at the beginning //

    // Create NavX Motion Processor for robot orientation and positioning //
    
    // Intialize all gyro readings to 0 //

    // Create Encoder for wheel rotation information //

    // Intialize all encoder readings to 0 //
    
  }

  // Drive Modes //
  public void arcadeDrive(double speed, double rotation) {
    m_drive.arcadeDrive(speed, rotation);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
