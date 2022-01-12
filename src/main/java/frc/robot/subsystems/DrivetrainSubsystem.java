// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
  
  
  // Declare subsystem attribute/components //
  
  // Motor Controllers //

  // Encoders //
  
  // NavX //
  
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

  }

  public void tankDrive(double leftSpeed, double rightSpeed) {

  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
