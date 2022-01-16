// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;

public class DrivetrainSubsystem extends SubsystemBase {
  
  
  // Declare subsystem attribute/components //
  
  // Motor Controllers //
  WPI_TalonFX m_leftLead = new WPI_TalonFX(2);
  WPI_TalonFX m_leftFollow = new WPI_TalonFX(1);
  WPI_TalonFX m_rightLead = new WPI_TalonFX(4);
  WPI_TalonFX m_rightFollow = new WPI_TalonFX(3);

  DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftLead, m_rightLead);

  // Encoders //
  
  // NavX //
  AHRS ahrs = new AHRS(SPI.Port.kMXP); 

  /** Creates a new Drivetrain. */
  public DrivetrainSubsystem() {

    // Create TalonFX Motor Controller for motor controls //

    // Initialize TalonFX to factory default //
    m_leftLead.configFactoryDefault();
    m_leftFollow.configFactoryDefault();
    m_rightLead.configFactoryDefault();
    m_rightFollow.configFactoryDefault();

    // Set followers to follow lead //
    m_leftFollow.follow(m_leftLead);
    m_rightFollow.follow(m_rightLead);

    // Flip values so robot moves forward when stick-forward/LEDs-green //
    m_leftLead.setInverted(TalonFXInvertType.Clockwise);
    m_rightLead.setInverted(TalonFXInvertType.CounterClockwise);

    // Set the followers to match their respective motor leads //
    m_leftFollow.setInverted(TalonFXInvertType.FollowMaster);
    m_rightFollow.setInverted(TalonFXInvertType.FollowMaster);

    // Create NavX Motion Processor for robot orientation and positioning //
    
    // Intialize all gyro readings to 0 //

    // Create Encoder for wheel rotation information //

    // Intialize all encoder readings to 0 //
    
  }

  // Drive Modes //
  public void arcadeDrive(double speed, double rotation) {
    m_diffDrive.arcadeDrive(speed, rotation);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_diffDrive.tankDrive(leftSpeed, rightSpeed);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
