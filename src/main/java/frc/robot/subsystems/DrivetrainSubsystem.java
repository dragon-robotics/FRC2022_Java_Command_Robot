// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;

public class DrivetrainSubsystem extends SubsystemBase {
  
  
  // Declare subsystem attribute/components //
  
  // Motor Controllers //
  WPI_TalonFX m_leftLead = new WPI_TalonFX(2);
  WPI_TalonFX m_leftFollow = new WPI_TalonFX(1);
  WPI_TalonFX m_rightLead = new WPI_TalonFX(4);
  WPI_TalonFX m_rightFollow = new WPI_TalonFX(3);

  DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftLead, m_rightLead);
  
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

    // Disable all motors //
    m_leftLead.set(ControlMode.PercentOutput, 0);
    m_leftFollow.set(ControlMode.PercentOutput, 0);
    m_rightLead.set(ControlMode.PercentOutput, 0);
    m_rightFollow.set(ControlMode.PercentOutput, 0);

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

    // Set neutral mode to brake on all motors //
    m_leftLead.setNeutralMode(NeutralMode.Coast);
    m_leftFollow.setNeutralMode(NeutralMode.Coast);
    m_rightLead.setNeutralMode(NeutralMode.Coast);
    m_rightFollow.setNeutralMode(NeutralMode.Coast);

    // Reset encoders to 0 //
    resetEncoders();
    
    // Intialize all gyro readings to 0 //
    zeroHeading();

    // Get TalonFX Integrated Sensor Values
    // Source: https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20Talon%20FX%20(Falcon%20500)/IntegratedSensor/src/main/java/frc/robot/Robot.java//
    //  Lines 83 - 87
    // TalonFXConfiguruation configs = new TalonFXConfiguration(); 
    // /* select integ-sensor for PID0 (it doesn't matter if PID is actually used) */
    // configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    // /* config all the settings, only need one left and right motors */
    // m_leftLead.configAllSettings(configs);
    // m_rightLead.configAllSettings(configs);

    // Initialize Robot Odometry //
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
        m_gyro.getRotation2d(), getDistance(m_leftEncoder), -getDistance(m_rightEncoder));

    // Added code to record X and Y odometry data //
    var translation = m_odometry.getPoseMeters().getTranslation();
    m_xEntry.setNumber(translation.getX());
    m_yEntry.setNumber(translation.getY());

    double degree = getHeading();
    m_angleEntry.setDouble(degree);

    // Output raw encoder values //
    m_leftEncoderEntry.setDouble(m_leftEncoder.getIntegratedSensorPosition());
    m_rightEncoderEntry.setDouble(m_rightEncoder.getIntegratedSensorPosition());

    // Output encoder values converted to distance //
    m_leftEncoderDistanceEntry.setDouble(getDistance(m_leftEncoder));
    m_rightEncoderDistanceEntry.setDouble(-getDistance(m_rightEncoder));

    // Output raw encoder velocity values //
    m_leftEncoderVelocityEntry.setDouble(m_leftEncoder.getIntegratedSensorVelocity());
    m_rightEncoderVelocityEntry.setDouble(-m_rightEncoder.getIntegratedSensorVelocity());

    // Output raw wheel speed values //
    m_leftEncoderWheelSpeedEntry.setDouble(getWheelSpeeds().leftMetersPerSecond);
    m_rightEncoderWheelSpeedEntry.setDouble(getWheelSpeeds().rightMetersPerSecond);

  }

  // Drive Modes //
  public void arcadeDrive(double speed, double rotation) {
    m_drive.arcadeDrive(speed / 2, rotation / 2);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_diffDrive.tankDrive(leftSpeed, rightSpeed);
  }
  
  public void tankDriveVolts(double leftVolts, double rightVolts){
    m_motorL.setVoltage(leftVolts);   // Set voltage for left motor
    m_motorR.setVoltage(rightVolts);  // Set voltage for right motor
    m_drive.feed();                   // Feed the motor safety object, stops the motor if anything goes wrong
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){

    double leftSpeed = m_leftEncoder.getIntegratedSensorVelocity() * Constants.ENCODER_DISTANCE_PER_PULSE * 10;
    double rightSpeed = -m_rightEncoder.getIntegratedSensorVelocity() * Constants.ENCODER_DISTANCE_PER_PULSE * 10;  // Need to invert the results
    return new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);
  }

  // Encoder Controls/Readings //
  public void resetEncoders(){
    m_leftEncoder.setIntegratedSensorPosition(0.0, 100);
    m_rightEncoder.setIntegratedSensorPosition(0.0, 100);
  }

  public double getDistance(TalonFXSensorCollection encoder){
    return encoder.getIntegratedSensorPosition() * Constants.ENCODER_DISTANCE_PER_PULSE;
  }

  // Gets the average encoder distance in meters //
  public double getAverageEncoderDistance() {
    return (getDistance(m_leftEncoder) + -getDistance(m_rightEncoder)) / 2.0;
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
