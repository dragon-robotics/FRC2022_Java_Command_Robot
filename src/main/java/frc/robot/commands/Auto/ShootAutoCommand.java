// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.General.ShooterMotorBackwardCommand;
import frc.robot.commands.General.ShooterMotorForwardCommand;
import frc.robot.commands.General.UptakeMotorDownCommand;
import frc.robot.commands.General.UptakeMotorUpCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.UptakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootAutoCommand extends SequentialCommandGroup {
  /** Creates a new ShootCommand. */
  public ShootAutoCommand(
    UptakeSubsystem uptake,
    ShooterSubsystem shooter
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelRaceGroup(
        new WaitCommand(0.3), // Have the uptake go down for half a second
        new UptakeMotorDownCommand(uptake),
        new ShooterMotorBackwardCommand(shooter)
      ),
      new ParallelCommandGroup(
        new UptakeMotorUpCommand(uptake),
        new ShooterMotorForwardCommand(shooter)
      )
    );
  }
}
