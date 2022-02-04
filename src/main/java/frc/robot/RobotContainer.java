// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems //
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  // Joystick - 1st driver (driver) = channel 0, 2nd driver (operator) = channel 1 //
  private final Joystick m_driverController = new Joystick(Constants.DRIVER);
  // private final Joystick m_operatorController = new Joystick(Constants.OPERATOR);

  // Auto-Only Commands //

  // Store our overall trajectory //
  Trajectory trajectory = new Trajectory();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    String fourBallAutoTraj1Json = "Four-Ball-Auto-Pt1.wpilib.json";
    String fourBallAutoTraj2Json = "Four-Ball-Auto-Pt2.wpilib.json";
    String fourBallAutoTraj3Json = "Four-Ball-Auto-Pt3.wpilib.json";
    String fourBallAutoTraj4Json = "Four-Ball-Auto-Pt4.wpilib.json";

    String FiveBallAutoTraj1Json = "Five-Ball-Auto-Pt1.wpilib.json";
    String FiveBallAutoTraj2Json = "Five-Ball-Auto-Pt2.wpilib.json";
    String FiveBallAutoTraj3Json = "Five-Ball-Auto-Pt3.wpilib.json";
    String FiveBallAutoTraj4Json = "Five-Ball-Auto-Pt4.wpilib.json";
    String FiveBallAutoTraj5Json = "Five-Ball-Auto-Pt5.wpilib.json";

    String FiveBallAuto2Traj1Json = "Five-Ball-Auto-2-Pt1.wpilib.json";
    String FiveBallAuto2Traj2Json = "Five-Ball-Auto-2-Pt2.wpilib.json";

    // Set default command to arcade drive when in teleop
    m_drivetrainSubsystem.setDefaultCommand(getArcadeDriveCommand());

    try {
      // Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(threeBallAutoTraj1Json);
      // Trajectory traj1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      // trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(threeBallAutoTraj2Json);
      // Trajectory traj2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      // trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(threeBallAutoTraj3Json);
      // Trajectory traj3 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      // Trajectory tempTrajectory = traj1.concatenate(traj2);
      // trajectory = tempTrajectory.concatenate(traj3);

      // Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(FiveBallAutoTraj1Json);
      // Trajectory traj1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      // trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(FiveBallAutoTraj2Json);
      // Trajectory traj2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      // trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(FiveBallAutoTraj3Json);
      // Trajectory traj3 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      // trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(FiveBallAutoTraj4Json);
      // Trajectory traj4 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      // trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(FiveBallAutoTraj5Json);
      // Trajectory traj5 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(FiveBallAuto2Traj1Json);
      Trajectory traj1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(FiveBallAuto2Traj2Json);
      Trajectory traj2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

      trajectory = traj1.concatenate(traj2);
    } catch (IOException ex) {

    }

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return m_autoCommand;
    return getRamseteCommand();
  }

  public Command getArcadeDriveCommand(){
    // Commands //
    return new ArcadeDriveCommand(
      m_drivetrainSubsystem,
      () -> -m_driverController.getRawAxis(Constants.STICK_LEFT_Y),
      () -> m_driverController.getRawAxis(Constants.STICK_RIGHT_X)
    );
  }

  public Command getRamseteCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
          Constants.ksVolts,
          Constants.kvVoltSecondsPerMeter,
          Constants.kaVoltSecondsSquaredPerMeter),
      Constants.kDriveKinematics,
      10
    );

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
      Constants.kMaxSpeedMetersPerSecond,
      Constants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Constants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //   // Start at the origin facing the +X direction
    //   new Pose2d(0, 0, new Rotation2d(0)),
    //   // Pass through these two interior waypoints, making an 's' curve path
    //   List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //   // End 3 meters straight ahead of where we started, facing forward
    //   new Pose2d(3, 0, new Rotation2d(0)),
    //   // Pass config
    //   config
    // );

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(
        new Translation2d(1, 2),
        new Translation2d(3, 1),
        new Translation2d(2, 0),
        new Translation2d(3, -1),
        new Translation2d(1, -2)
      ),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(0, 0, new Rotation2d(Math.toRadians(-180))),
      // Pass config
      config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
      // exampleTrajectory,
      trajectory,
      m_drivetrainSubsystem::getPose,
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      new SimpleMotorFeedforward(
          Constants.ksVolts,
          Constants.kvVoltSecondsPerMeter,
          Constants.kaVoltSecondsSquaredPerMeter),
      Constants.kDriveKinematics,
      m_drivetrainSubsystem::getWheelSpeeds,
      new PIDController(Constants.kPDriveVel, 0, 0),
      new PIDController(Constants.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      m_drivetrainSubsystem::tankDriveVolts,
      m_drivetrainSubsystem
    );

    // Reset odometry to the starting pose of the trajectory.
    m_drivetrainSubsystem.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_drivetrainSubsystem.tankDriveVolts(0, 0));
  }
}
