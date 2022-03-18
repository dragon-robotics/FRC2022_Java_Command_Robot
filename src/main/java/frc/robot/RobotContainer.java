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
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Teleop.ArcadeDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.UptakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.AutoLoader.AutoCommand;
import frc.robot.commands.Auto.FiveBallBotLowGoalCommand;
import frc.robot.commands.Auto.FourBallTopLeftLowGoalCommand;
import frc.robot.commands.Auto.IntakeCargoAutoCommand;
import frc.robot.commands.Auto.OneBallBotLeftLowGoalCommand;
import frc.robot.commands.Auto.OneBallBotLowGoalCommand;
import frc.robot.commands.Auto.OneBallTopLeftLowGoalCommand;
import frc.robot.commands.Auto.OneBallTopLowGoalCommand;
import frc.robot.commands.Auto.ShootAutoCommand;
import frc.robot.commands.Auto.TwoBallBotLeftLowGoalCommand;
import frc.robot.commands.Auto.TwoBallBotLowGoalCommand;
import frc.robot.commands.Auto.TwoBallTopLeftLowGoalCommand;
import frc.robot.commands.Auto.TwoBallTopLowGoalCommand;
import frc.robot.commands.General.IntakeCompressorOffCommand;
import frc.robot.commands.General.IntakeCompressorOnCommand;
import frc.robot.commands.General.IntakeNeutralCommand;
import frc.robot.commands.General.VariableShootCommand;
import frc.robot.commands.General.VariableUptakeCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems //
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final UptakeSubsystem m_uptakeSubsystem = new UptakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  // Joystick - 1st driver (driver) = channel 0, 2nd driver (operator) = channel 1 //
  private final Joystick m_driverController = new Joystick(Constants.DRIVER);

  private final Joystick m_operatorController = new Joystick(Constants.OPERATOR);
  private final JoystickButton m_intakeCompressorOffButton = new JoystickButton(m_operatorController, Constants.BTN_BACK);
  private final JoystickButton m_intakeCompressorOnButton = new JoystickButton(m_operatorController, Constants.BTN_START);
  private final JoystickButton m_intakeCargoButton = new JoystickButton(m_operatorController, Constants.BTN_A);
  private final JoystickButton m_shootButton = new JoystickButton(m_operatorController, Constants.BTN_B);

  // Create the auto loader class to load everything for us //
  private final AutoLoader m_autoLoader = new AutoLoader();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Set default command to arcade drive when in teleop
    m_drivetrainSubsystem.setDefaultCommand(
      new ArcadeDriveCommand(
        m_drivetrainSubsystem,
        () -> -m_driverController.getRawAxis(Constants.STICK_LEFT_Y),  // speed
        () -> m_driverController.getRawAxis(Constants.STICK_RIGHT_X),  // turn
        () -> m_driverController.getRawAxis(Constants.TRIGGER_LEFT),   // throttle
        () -> m_driverController.getRawButton(Constants.BUMPER_LEFT)   // reverse
      )
    );
    
    // Set default intake to have neutral motor and intake //
    // m_intakeSubsystem.setDefaultCommand(
    //   new IntakePistonNeutralCommand(m_intakeSubsystem)
    // );

    // m_intakeSubsystem.setDefaultCommand(
    //   new IntakeTest(
    //     m_intakeSubsystem,
    //     () -> m_operatorController.getRawAxis(Constants.STICK_LEFT_X),    // speed
    //     () -> m_operatorController.getRawButton(Constants.BUMPER_RIGHT),  // extend
    //     () -> m_operatorController.getRawButton(Constants.BUMPER_LEFT)    // retract
    //   )
    // );

    m_intakeSubsystem.setDefaultCommand(
      new IntakeNeutralCommand(m_intakeSubsystem)
    );

    m_uptakeSubsystem.setDefaultCommand(
      new VariableUptakeCommand(
        m_uptakeSubsystem,
        () -> m_operatorController.getRawAxis(Constants.TRIGGER_RIGHT),
        () -> -m_operatorController.getRawAxis(Constants.TRIGGER_LEFT)
      )
    );

    m_shooterSubsystem.setDefaultCommand(
      new VariableShootCommand(
        m_shooterSubsystem,
        () -> m_operatorController.getRawAxis(Constants.STICK_RIGHT_Y)
      )
    );

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
  private void configureButtonBindings() {
    m_intakeCompressorOffButton.whenPressed(new IntakeCompressorOffCommand(m_intakeSubsystem));
    m_intakeCompressorOnButton.whenPressed(new IntakeCompressorOnCommand(m_intakeSubsystem));
    m_intakeCargoButton.whenHeld(new IntakeCargoAutoCommand(m_intakeSubsystem, m_uptakeSubsystem));
    m_shootButton.whenHeld(new ShootAutoCommand(m_uptakeSubsystem, m_shooterSubsystem));
  }

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
            m_drivetrainSubsystem, m_intakeSubsystem, m_uptakeSubsystem, m_shooterSubsystem, command);
      case ONE_BALL_TOP_LEFT_LOW_GOAL:
        return new OneBallTopLeftLowGoalCommand(
            m_drivetrainSubsystem, m_intakeSubsystem, m_uptakeSubsystem, m_shooterSubsystem, command);
      case ONE_BALL_BOT_LEFT_LOW_GOAL:
        return new OneBallBotLeftLowGoalCommand(
            m_drivetrainSubsystem, m_intakeSubsystem, m_uptakeSubsystem, m_shooterSubsystem, command);
      case ONE_BALL_BOT_LOW_GOAL:
        return new OneBallBotLowGoalCommand(
            m_drivetrainSubsystem, m_intakeSubsystem, m_uptakeSubsystem, m_shooterSubsystem, command);
      case TWO_BALL_TOP_LOW_GOAL:
        return new TwoBallTopLowGoalCommand(
            m_drivetrainSubsystem, m_intakeSubsystem, m_uptakeSubsystem, m_shooterSubsystem, command);
      case TWO_BALL_TOP_LEFT_LOW_GOAL:
        return new TwoBallTopLeftLowGoalCommand(
            m_drivetrainSubsystem, m_intakeSubsystem, m_uptakeSubsystem, m_shooterSubsystem, command);
      case TWO_BALL_BOT_LEFT_LOW_GOAL:
        return new TwoBallBotLeftLowGoalCommand(
            m_drivetrainSubsystem, m_intakeSubsystem, m_uptakeSubsystem, m_shooterSubsystem, command);
      case TWO_BALL_BOT_LOW_GOAL:
        return new TwoBallBotLowGoalCommand(
            m_drivetrainSubsystem, m_intakeSubsystem, m_uptakeSubsystem, m_shooterSubsystem, command);
      case FOUR_BALL_TOP_LEFT_LOW_GOAL:
        return new FourBallTopLeftLowGoalCommand(
            m_drivetrainSubsystem, command);
      case FIVE_BALL_BOT_LOW_GOAL:
        return new FiveBallBotLowGoalCommand(
            m_drivetrainSubsystem, command);
      default:
        return null;
    }
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
