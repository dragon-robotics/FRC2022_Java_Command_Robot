// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;

public class DrivetrainSubsystem extends SubsystemBase {
  
  
  // Declare subsystem attribute/components //
  
  // Motor Controllers //
  private final WPI_TalonFX m_talonLeftLead = new WPI_TalonFX(2);
  private final WPI_TalonFX m_talonLeftFollow = new WPI_TalonFX(1);
  private final WPI_TalonFX m_talonRightLead = new WPI_TalonFX(4);
  private final WPI_TalonFX m_talonRightFollow = new WPI_TalonFX(3);
  private final DifferentialDrive m_drive = new DifferentialDrive(m_talonLeftLead, m_talonRightLead);
  
  // Gyro - NavX //
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Encoders - REV Through-bore //
  private final Encoder m_leftEncoder = new Encoder(
    Constants.LEFT_ENCODER_PORTS[0],
    Constants.LEFT_ENCODER_PORTS[1],
    Constants.LEFT_ENCODER_REVERSED
  );
  private final Encoder m_rightEncoder = new Encoder(
    Constants.RIGHT_ENCODER_PORTS[0],
    Constants.RIGHT_ENCODER_PORTS[1],
    Constants.RIGHT_ENCODER_REVERSED
  );

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
    m_talonLeftLead.setNeutralMode(NeutralMode.Coast);
    m_talonLeftFollow.setNeutralMode(NeutralMode.Coast);
    m_talonRightLead.setNeutralMode(NeutralMode.Coast);
    m_talonRightFollow.setNeutralMode(NeutralMode.Coast);

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
    m_leftEncoder.setDistancePerPulse(Constants.ENCODER_DISTANCE_PER_PULSE);
    m_rightEncoder.setDistancePerPulse(Constants.ENCODER_DISTANCE_PER_PULSE);

    // Set encoder samples to 10 //
    m_leftEncoder.setSamplesToAverage(Constants.ENCODER_SAMPLES_PER_AVG);

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
        m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

    // Added code to record X and Y odometry data //
    var translation = m_odometry.getPoseMeters().getTranslation();
    m_xEntry.setNumber(translation.getX());
    m_yEntry.setNumber(translation.getY());

    double degree = getHeading();
    m_angleEntry.setDouble(degree);

    // Output raw encoder values //
    m_leftEncoderEntry.setDouble(m_leftEncoder.get());
    m_rightEncoderEntry.setDouble(m_rightEncoder.get());

    // Output encoder values converted to distance //
    m_leftEncoderDistanceEntry.setDouble(m_leftEncoder.getDistance());
    m_rightEncoderDistanceEntry.setDouble(m_rightEncoder.getDistance());

    // Output raw encoder velocity values //
    m_leftEncoderVelocityEntry.setDouble(m_leftEncoder.getRate());
    m_rightEncoderVelocityEntry.setDouble(m_rightEncoder.getRate());

    // Output raw wheel speed values //
    m_leftEncoderWheelSpeedEntry.setDouble(getWheelSpeeds().leftMetersPerSecond);
    m_rightEncoderWheelSpeedEntry.setDouble(getWheelSpeeds().rightMetersPerSecond);

  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement (-1.0, 1.0)
   * @param rot the commanded rotation (-1.0, 1.0)
   */
  public void arcadeDrive(double speed, double rotation) {
    m_drive.arcadeDrive(speed / 2, rotation / 2);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftSpeed  the commanded left speed (-1.0, 1.0)
   * @param rightSpeed the commanded right output (-1.0, 1.0)
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }
  
  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output (-12V, 12V)
   * @param rightVolts the commanded right output (-12V, 12V)
   */
  public void tankDriveVolts(double leftVolts, double rightVolts){
    m_talonLeftLead.setVoltage(leftVolts);   // Set voltage for left motor
    m_talonRightLead.setVoltage(rightVolts);  // Set voltage for right motor
    m_drive.feed();                   // Feed the motor safety object, stops the motor if anything goes wrong
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  // Encoder Controls/Readings //
  public void resetEncoders(){
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Gets the average distance of the encoders of the left and right side
   *
   * @return the average distance of both left and right encoder
   */  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return m_rightEncoder;
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

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose){
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }
}
