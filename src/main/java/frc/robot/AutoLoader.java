// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class AutoLoader {

    /**
     * Enumeration for the possible autos that we'll create
     */
    public enum AutoCommand {
        NONE, // Does Nothing
        EXAMPLE_TRAJECTORY, // Executes the Example Trajectory for Testing
        LEAVE_TARMAC, // Just leaves the tarmac without scoring
        // LOW GOAL //
        ONE_BALL_TOP_LOW_GOAL, // Scores the 1 ball on the low goal from the top side of the tarmac
        ONE_BALL_TOP_LEFT_LOW_GOAL, // Scores the 1 ball on the low goal from the top left side of the tarmac
        ONE_BALL_BOT_LEFT_LOW_GOAL, // Scores the 1 ball on the low goal from the bottom left side of the tarmac
        ONE_BALL_BOT_LOW_GOAL, // Scores the 1 ball on the low goal from the bottom side of the tarmac
        TWO_BALL_TOP_LOW_GOAL, // Picks up the 1 ball from the top side and scores the low goal
        TWO_BALL_TOP_LEFT_LOW_GOAL, // Picks up the 1 ball from the top left side and scores the low goal
        TWO_BALL_BOT_LEFT_LOW_GOAL, // Picks up the 1 ball from the bottom left side and scores the low goal
        TWO_BALL_BOT_LOW_GOAL, // Picks up the 1 ball from the bottom side and scores the low goal
        THREE_BALL_TOP_LEFT_LOW_GOAL, // Picks up the 1 ball from the top and score low
        THREE_BALL_BOT_LEFT_LOW_GOAL, // Picks up the 1 ball from the bottom left and score low
        THREE_BALL_BOT_LOW_GOAL, // Picks up the 1 ball from the bottom and score low
        FOUR_BALL_TOP_LEFT_LOW_GOAL, // Scores the 1 ball on the robot, drives to terminal, pick up 2 balls, score 2
                                     // low
        FOUR_BALL_TOP_LOW_GOAL, // Picks up ball from bottom left, score 2 low, pick up 2 left ball, score 2 low
        FOUR_BALL_BOT_LEFT_LOW_GOAL, // Picks up ball from bottom left, score 2 low, pick up 2 left ball, score 2 low
        FOUR_BALL_BOT_LOW_GOAL, // Picks up ball from bottom, score 2 low, pick up 2 left ball, score 2 low
        FIVE_BALL_BOT_LOW_GOAL, // Picks up ball from bottom, score 2 low, pick up 1 left ball, score 1 low,
                                // pick up 2 left terminal balls, score 2 low
        // HIGH GOAL //
        ONE_BALL_TOP_HIGH_GOAL, // Scores the 1 ball on the high goal from the top side of the tarmac
        ONE_BALL_TOP_LEFT_HIGH_GOAL, // Scores the 1 ball on the high goal from the top left side of the tarmac
        ONE_BALL_BOT_LEFT_HIGH_GOAL, // Scores the 1 ball on the high goal from the bottom left side of the tarmac
        ONE_BALL_BOT_HIGH_GOAL, // Scores the 1 ball on the high goal from the bottom side of the tarmac
        TWO_BALL_TOP_HIGH_GOAL, // Picks up the lone ball from the top side and scores the high goal
        TWO_BALL_TOP_LEFT_HIGH_GOAL, // Picks up the lone ball from the top left side and scores the high goal
        TWO_BALL_BOT_LEFT_HIGH_GOAL, // Picks up the lone ball from the bottom left side and scores the high goal
        TWO_BALL_BOT_HIGH_GOAL, // Picks up the lone ball from the bottom side and scores the high goal
        THREE_BALL_BOT_LEFT_HIGH_GOAL, // Picks up the one ball from the bottom left and score high
        THREE_BALL_BOT_HIGH_GOAL, // Picks up the one ball from the bottom left and score high
        FOUR_BALL_BOT_LEFT_HIGH_GOAL, // Picks up ball from bottom left, score 2 high, pick up 2 left ball, score 2
                                      // high
        FOUR_BALL_BOT_HIGH_GOAL, // Picks up ball from bottom, score 2 high, pick up 2 left ball, score 2 high
        FIVE_BALL_BOT_HIGH_GOAL // Score high, pick up bottom and the nearest left ball, score 2 high, pick up
                                // leftmost and human player ball, score 2 high
    }

    private SendableChooser<AutoCommand> m_autoChooser;

    /**
     * Constructor used to initialize the sendable chooser
     */
    public AutoLoader() {
        // Determine which alliance we're in to determine if we need to mirror and
        // invert the trajectories //

        // Initialize the sendable chooser //
        m_autoChooser = new SendableChooser<>();

        // Default option is to always have no auto command running //
        m_autoChooser.setDefaultOption("None", AutoCommand.NONE);

        // Initialize the rest of the options //
        m_autoChooser.addOption("Example Trajectory", AutoCommand.EXAMPLE_TRAJECTORY);
        m_autoChooser.addOption("Leave Tarmac", AutoCommand.LEAVE_TARMAC);
        m_autoChooser.addOption("One Ball Top Low Goal", AutoCommand.ONE_BALL_TOP_LOW_GOAL);
        m_autoChooser.addOption("One Ball Top Left Low Goal", AutoCommand.ONE_BALL_TOP_LEFT_LOW_GOAL);
        m_autoChooser.addOption("One Ball Bottom Left Low Goal", AutoCommand.ONE_BALL_BOT_LEFT_LOW_GOAL);
        m_autoChooser.addOption("One Ball Bottom Low Goal", AutoCommand.ONE_BALL_BOT_LOW_GOAL);
        m_autoChooser.addOption("Two Ball Top Low Goal", AutoCommand.TWO_BALL_TOP_LOW_GOAL);
        m_autoChooser.addOption("Two Ball Top Left Low Goal", AutoCommand.TWO_BALL_TOP_LEFT_LOW_GOAL);
        m_autoChooser.addOption("Two Ball Bottom Low Goal", AutoCommand.TWO_BALL_BOT_LOW_GOAL);
        m_autoChooser.addOption("Two Ball Bottom Left Low Goal", AutoCommand.TWO_BALL_BOT_LEFT_LOW_GOAL);
        m_autoChooser.addOption("Four Ball Top Left Low Goal", AutoCommand.FOUR_BALL_TOP_LEFT_LOW_GOAL);
        m_autoChooser.addOption("Five Ball Bot Low Goal", AutoCommand.FIVE_BALL_BOT_LOW_GOAL);

        SmartDashboard.putData(m_autoChooser);
    }

    /**
     * 
     * @return The selected command from the m_autoChooser
     */
    public AutoCommand getSelected() {
        return m_autoChooser.getSelected();
    }

}
