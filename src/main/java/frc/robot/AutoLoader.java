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
        NONE,                           // Does Nothing
        EXAMPLE_TRAJECTORY,             // Executes the Example Trajectory for Testing
        LEAVE_TARMAC,                   // Just leaves the tarmac without scoring
        ONE_BALL_LOW_GOAL,              // Scores the 1 ball on the low goal
        ONE_BALL_HIGH_GOAL,             // Scores the 1 ball on the high goal
        TWO_BALL_TOP_SIDE_LOW_GOAL,     // Picks up the lone ball from the top side and scores the low goal
        TWO_BALL_TOP_SIDE_HIGH_GOAL,    // Picks up the lone ball from the top side and scores the high goal
        TWO_BALL_BOT_SIDE_LOW_GOAL,     // Picks up the ball from the bot side and scores the low goal
        TWO_BALL_BOT_SIDE_HIGH_GOAL,    // Picks up the ball from the bot side and scores the high goal
        TWO_BALL_LEFT_SIDE_LOW_GOAL,    // Picks up the ball from the lft side and scores the low goal
        TWO_BALL_LEFT_SIDE_HIGH_GOAL,   // Picks up the ball from the lft side and scores the high goal
        THREE_BALL_BOT_SIDE_LOW_GOAL,   // Picks up the one ball from the bottom, score 2 low, then grab the closest ball to the left and score low
        THREE_BALL_BOT_SIDE_HIGH_GOAL,  // Picks up the one ball from the bottom, score 2 high, then grab the closest ball to the left and score high
        FOUR_BALL_LOW_GOAL,             // Pick up bottom ball, score 2 low, pick up 2 left ball, score 2 low
        FOUR_BALL_HIGH_GOAL,            // Pick up bottom ball, score 2 high, pick up 2 left ball, score 2 high
        FIVE_BALL_LOW_GOAL,             // Score low, pick up bottom and the nearest left ball, score 2 low, pick up leftmost and human player ball, score 2 low
        FIVE_BALL_HIGH_GOAL             // Score high, pick up bottom and the nearest left ball, score 2 high, pick up leftmost and human player ball, score 2 high
    }

    private SendableChooser<AutoCommand> m_autoChooser;

    /**
     * Constructor used to initialize the sendable chooser
     */
    public AutoLoader(){
        // Determine which alliance we're in to determine if we need to mirror and invert the trajectories //

        // Initialize the sendable chooser //
        m_autoChooser = new SendableChooser<>();

        // Default option is to always have no auto command running //
        m_autoChooser.setDefaultOption("None", AutoCommand.NONE);

        // Initialize the rest of the options //
        m_autoChooser.addOption("Example Trajectory", AutoCommand.EXAMPLE_TRAJECTORY);
        m_autoChooser.addOption("Leave Tarmac", AutoCommand.LEAVE_TARMAC);
        m_autoChooser.addOption("One Ball Low Goal", AutoCommand.ONE_BALL_LOW_GOAL);
        m_autoChooser.addOption("One Ball High Goal", AutoCommand.ONE_BALL_HIGH_GOAL);
        m_autoChooser.addOption("Two Ball Top Side Low Goal", AutoCommand.TWO_BALL_TOP_SIDE_LOW_GOAL);
        m_autoChooser.addOption("Two Ball Top Side High Goal", AutoCommand.TWO_BALL_TOP_SIDE_HIGH_GOAL);
        m_autoChooser.addOption("Two Ball Bot Side Low Goal", AutoCommand.TWO_BALL_BOT_SIDE_LOW_GOAL);
        m_autoChooser.addOption("Two Ball Bot Side High Goal", AutoCommand.TWO_BALL_BOT_SIDE_HIGH_GOAL);
        m_autoChooser.addOption("Two Ball Left Side Low Goal", AutoCommand.TWO_BALL_LEFT_SIDE_LOW_GOAL);
        m_autoChooser.addOption("Two Ball Left Side High Goal", AutoCommand.TWO_BALL_LEFT_SIDE_HIGH_GOAL);                
        m_autoChooser.addOption("Three Ball Bot Side Low Goal", AutoCommand.THREE_BALL_BOT_SIDE_LOW_GOAL);
        m_autoChooser.addOption("Three Ball Bot Side High Goal", AutoCommand.THREE_BALL_BOT_SIDE_HIGH_GOAL);
        m_autoChooser.addOption("Four Ball Low Goal", AutoCommand.FOUR_BALL_LOW_GOAL);
        m_autoChooser.addOption("Four Ball High Goal", AutoCommand.FOUR_BALL_HIGH_GOAL);
        m_autoChooser.addOption("Five Ball Low Goal", AutoCommand.FIVE_BALL_LOW_GOAL);
        m_autoChooser.addOption("Five Ball High Goal", AutoCommand.FIVE_BALL_HIGH_GOAL);

        SmartDashboard.putData(m_autoChooser);
    }

    /**
     * 
     * @return The selected command from the m_autoChooser
     */
    public AutoCommand getSelected(){
        return m_autoChooser.getSelected();
    }


}
