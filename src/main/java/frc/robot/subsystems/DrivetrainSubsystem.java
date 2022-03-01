// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;

public class DrivetrainSubsystem extends SubsystemBase {
  
  
  // Declare subsystem attribute/components //
  
  // Motor Controllers //
  WPI_TalonFX m_talonLeftLead = new WPI_TalonFX(Constants.TALONFX_LEFT_TOP);
  WPI_TalonFX m_talonLeftFollow = new WPI_TalonFX(Constants.TALONFX_LEFT_BOTTOM);
  WPI_TalonFX m_talonRightLead = new WPI_TalonFX(Constants.TALONFX_RIGHT_TOP);
  WPI_TalonFX m_talonRightFollow = new WPI_TalonFX(Constants.TALONFX_RIGHT_BOTTOM);
  DifferentialDrive m_drive = new DifferentialDrive(m_talonLeftLead, m_talonRightLead);
  
  // Gyro - NavX //
  AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robotpose
  private final DifferentialDriveOdometry m_odometry;

  // This network table is setup to track the robot's position after each X and Y update //
  NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
  NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");
  NetworkTableEntry m_angleEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Angle");
  NetworkTableEntry m_leftEncoderEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Left Encoder");
  NetworkTableEntry m_rightEncoderEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Right Encoder");
  NetworkTableEntry m_leftEncoderDistanceEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Left Encoder Distance");
  NetworkTableEntry m_rightEncoderDistanceEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Right Encoder Distance");
  NetworkTableEntry m_leftEncoderVelocityEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Left Encoder Velocity");
  NetworkTableEntry m_rightEncoderVelocityEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Right Encoder Velocity");
  NetworkTableEntry m_leftEncoderWheelSpeedEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Left Encoder Wheel Speed");
  NetworkTableEntry m_rightEncoderWheelSpeedEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Right Encoder Wheel Speed");

  /** Creates a new Drivetrain. */
  public DrivetrainSubsystem() {

    // Factory default configurations for all motors //
    m_talonLeftLead.configFactoryDefault();
    m_talonLeftFollow.configFactoryDefault();
    m_talonRightLead.configFactoryDefault();
    m_talonRightFollow.configFactoryDefault();

    // Disable all motors //
    m_talonLeftLead.set(ControlMode.PercentOutput, 0);
    m_talonLeftFollow.set(ControlMode.PercentOutput, 0);
    m_talonRightLead.set(ControlMode.PercentOutput, 0);
    m_talonRightFollow.set(ControlMode.PercentOutput, 0);

    // Set neutral mode to brake on all motors //
    m_talonLeftLead.setNeutralMode(NeutralMode.Brake);
    m_talonLeftFollow.setNeutralMode(NeutralMode.Brake);
    m_talonRightLead.setNeutralMode(NeutralMode.Brake);
    m_talonRightFollow.setNeutralMode(NeutralMode.Brake);

    // Set our followers to follow the lead motor //
    m_talonLeftFollow.follow(m_talonLeftLead);
    m_talonRightFollow.follow(m_talonRightLead);

    // Set our follower's inverted to be opposite of the master //
    m_talonLeftFollow.setInverted(InvertType.FollowMaster);
    m_talonRightFollow.setInverted(InvertType.FollowMaster);

    // Set our lead motor's rotation orientations //
    m_talonLeftLead.setInverted(TalonFXInvertType.CounterClockwise);
    m_talonRightLead.setInverted(TalonFXInvertType.Clockwise);

    // Configure encoder readings on the TalonFX //
    m_talonLeftLead.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    m_talonRightLead.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    // Reset encoders to 0 //
    resetEncoders();
    
    // Intialize all gyro readings to 0 //
    zeroHeading();

    // Initialize Robot Odometry //
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
        m_gyro.getRotation2d(), getDistance(m_talonLeftLead), getDistance(m_talonRightLead));

    // Added code to record X and Y odometry data //
    var translation = m_odometry.getPoseMeters().getTranslation();
    m_xEntry.setNumber(translation.getX());
    m_yEntry.setNumber(translation.getY());

    double degree = getHeading();
    m_angleEntry.setDouble(degree);

    // Output raw encoder values //
    m_leftEncoderEntry.setDouble(m_talonLeftLead.getSelectedSensorPosition());
    m_rightEncoderEntry.setDouble(m_talonRightLead.getSelectedSensorPosition());

    // Output encoder values converted to distance //
    m_leftEncoderDistanceEntry.setDouble(getDistance(m_talonLeftLead));
    m_rightEncoderDistanceEntry.setDouble(getDistance(m_talonRightLead));

    // Output raw encoder velocity values //
    m_leftEncoderVelocityEntry.setDouble(m_talonLeftLead.getSelectedSensorVelocity());
    m_rightEncoderVelocityEntry.setDouble(m_talonRightLead.getSelectedSensorVelocity());

    // Output raw wheel speed values //
    m_leftEncoderWheelSpeedEntry.setDouble(getWheelSpeeds().leftMetersPerSecond);
    m_rightEncoderWheelSpeedEntry.setDouble(getWheelSpeeds().rightMetersPerSecond);

  }

  // Drive Modes //
  public void arcadeDrive(double speed, double rotation) {
    m_drive.arcadeDrive(speed, rotation);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }
  
  public void tankDriveVolts(double leftVolts, double rightVolts){
    m_talonLeftLead.setVoltage(leftVolts);   // Set voltage for left motor
    m_talonRightLead.setVoltage(rightVolts);  // Set voltage for right motor
    m_drive.feed();                   // Feed the motor safety object, stops the motor if anything goes wrong
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){

    double leftSpeed = m_talonLeftLead.getSelectedSensorVelocity() * Constants.ENCODER_DISTANCE_PER_PULSE * 10;
    double rightSpeed = m_talonRightLead.getSelectedSensorVelocity() * Constants.ENCODER_DISTANCE_PER_PULSE * 10;  // Need to invert the results
    return new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);
  }

  // Encoder Controls/Readings //
  public void resetEncoders(){
    m_talonLeftLead.setSelectedSensorPosition(0.0, 0, 0);
    m_talonRightLead.setSelectedSensorPosition(0.0, 0, 0);
  }

  public double getDistance(WPI_TalonFX motor){
    return motor.getSelectedSensorPosition() * Constants.ENCODER_DISTANCE_PER_PULSE;
  }

  // Gets the average encoder distance in meters //
  public double getAverageEncoderDistance() {
    return (getDistance(m_talonLeftLead) + getDistance(m_talonRightLead)) / 2.0;
  }

  // Gyro Controls/Readings //
  
  /** Zeroes the heading of the robot. */
  public void zeroHeading(){
    m_gyro.reset();   // Resets the Gyro
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }
}
