// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
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
  WPI_TalonFX m_talonLT = new WPI_TalonFX(2);
  WPI_TalonFX m_talonLB = new WPI_TalonFX(1);
  WPI_TalonFX m_talonRT = new WPI_TalonFX(4);
  WPI_TalonFX m_talonRB = new WPI_TalonFX(3);
  MotorControllerGroup m_motorL = new MotorControllerGroup(m_talonLT, m_talonLB);
  MotorControllerGroup m_motorR = new MotorControllerGroup(m_talonRT, m_talonRB);
  DifferentialDrive m_drive = new DifferentialDrive(m_motorL, m_motorR);

  // Encoders - TalonFX Integrated Motors //
  TalonFXSensorCollection m_leftEncoder = new TalonFXSensorCollection(m_talonLT);
  TalonFXSensorCollection m_rightEncoder = new TalonFXSensorCollection(m_talonRT);
  
  // Gyro - NavX //
  AHRS m_gyro = new AHRS(SPI.Port.kMXP); 

  // Odometry class for tracking robotpose
  private final DifferentialDriveOdometry m_odometry;

  // This network table is setup to track the robot's position after each X and Y update //
  NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
  NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");

  /** Creates a new Drivetrain. */
  public DrivetrainSubsystem() {

    // Disable all motors //
    m_talonLT.set(ControlMode.PercentOutput, 0);
    m_talonLB.set(ControlMode.PercentOutput, 0);
    m_talonRT.set(ControlMode.PercentOutput, 0);
    m_talonRB.set(ControlMode.PercentOutput, 0);

    // Factory default configurations for all motors //
    m_talonLT.configFactoryDefault();
    m_talonLB.configFactoryDefault();
    m_talonRT.configFactoryDefault();
    m_talonRB.configFactoryDefault();

    // Set neutral mode to brake on all motors //
    m_talonLT.setNeutralMode(NeutralMode.Coast);
    m_talonLB.setNeutralMode(NeutralMode.Coast);
    m_talonRT.setNeutralMode(NeutralMode.Coast);
    m_talonRB.setNeutralMode(NeutralMode.Coast);

    // Invert the right motors //
    m_motorR.setInverted(true);

    // Reset encoders to 0 //
    resetEncoders();

    // Calibrate and Reset NavX //
    m_gyro.calibrate();
    
    // Intialize all gyro readings to 0 //
    zeroHeading();

    // Get TalonFX Integrated Sensor Values
    // Source: https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20Talon%20FX%20(Falcon%20500)/IntegratedSensor/src/main/java/frc/robot/Robot.java//
    //  Lines 83 - 87
    // TalonFXConfiguruation configs = new TalonFXConfiguration(); 
    // /* select integ-sensor for PID0 (it doesn't matter if PID is actually used) */
    // configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    // /* config all the settings, only need one left and right motors */
    // m_talonLT.configAllSettings(configs);
    // m_talonRT.configAllSettings(configs);

    // Initialize Robot Odometry //
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
        m_gyro.getRotation2d(), getDistance(m_leftEncoder), getDistance(m_rightEncoder));

    // Added code to record X and Y odometry data //
    var translation = m_odometry.getPoseMeters().getTranslation();
    m_xEntry.setNumber(translation.getX());
    m_yEntry.setNumber(translation.getY()); 
  }

  // Drive Modes //
  public void arcadeDrive(double speed, double rotation) {
    m_drive.arcadeDrive(speed, rotation);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drive.tankDrive(leftSpeed, rightSpeed);
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
    double rightSpeed = m_rightEncoder.getIntegratedSensorVelocity() * Constants.ENCODER_DISTANCE_PER_PULSE * 10;
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
    return (getDistance(m_leftEncoder) + getDistance(m_rightEncoder)) / 2.0;
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
