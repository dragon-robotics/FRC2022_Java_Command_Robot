// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Teleop.ArcadeDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.AutoLoader.AutoCommand;
import frc.robot.commands.Auto.FourBallTopLeftLowGoalCommand;
import frc.robot.commands.Auto.OneBallTopLowGoalCommand;

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

  // Create the auto loader class to load everything for us //

  // Create SmartDashboard chooser for autonomous routines
  private final AutoLoader m_autoLoader = new AutoLoader();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Set default command to arcade drive when in teleop
    m_drivetrainSubsystem.setDefaultCommand(getArcadeDriveCommand());

    // Load all wpilib.json trajectory files into the Roborio to speed up auto
    // deployment //
    GenerateTrajectory.loadTrajectories();

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
    AutoCommand command = m_autoLoader.getSelected();

    switch (command) {
      case NONE:
        return null;
      case EXAMPLE_TRAJECTORY:
        return getRamseteCommand();
      case ONE_BALL_TOP_LOW_GOAL:
        return new OneBallTopLowGoalCommand(
          m_drivetrainSubsystem, command);
      case FOUR_BALL_TOP_LEFT_LOW_GOAL:
        return new FourBallTopLeftLowGoalCommand(
            m_drivetrainSubsystem, command);
      default:
        return null;
    }
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
    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = GenerateTrajectory.getTrajectory(AutoCommand.EXAMPLE_TRAJECTORY).get(0);

    RamseteCommand ramseteCommand = new RamseteCommand(
      exampleTrajectory,
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
    m_drivetrainSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_drivetrainSubsystem.tankDriveVolts(0, 0));
  }
}
