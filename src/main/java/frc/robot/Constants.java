// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Controller IDs, DRIVER = 0, OPERATOR = 1
    public final static int DRIVER = 0;
    public final static int OPERATOR = 1;

    // Joystick Mapping IDs //

    // Joystick Analog Axis/Stick //
    public final static int STICK_LEFT_X = 0;
    public final static int STICK_LEFT_Y = 1;
    public final static int TRIGGER_LEFT = 2;
    public final static int TRIGGER_RIGHT = 3;
    public final static int STICK_RIGHT_X = 4;
    public final static int STICK_RIGHT_Y = 5;

    // Joystick Buttons //
    public final static int BTN_A = 1;
    public final static int BTN_B = 2;
    public final static int BTN_X = 3;
    public final static int BTN_Y = 4;
    public final static int BUMPER_LEFT = 5;
    public final static int BUMPER_RIGHT = 6;
    public final static int BTN_BACK = 7;
    public final static int BTN_START = 8;
    public final static int BTN_STICK_LEFT = 9;
    public final static int BTN_STICK_RIGHT = 10;

    /* Feedforward Feedback Gain */
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondPerMeter = 1.98;
    public static final double kaVoltSecondSquaredPerMeter = 0.2;
    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;

    /* DifferentialDriveKinematics */
    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    /* Max Trajectory Velocity/Acceleration */
    public static final double kMaxSpeedMeteresPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    /* Ramsete Parameters */
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
