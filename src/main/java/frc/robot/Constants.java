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
    public static final int DRIVER = 0;
    public static final int OPERATOR = 1;

    // Joystick Analog Axis/Stick //
    public static final int STICK_LEFT_X = 0;
    public static final int STICK_LEFT_Y = 1;
    public static final int TRIGGER_LEFT = 2;
    public static final int TRIGGER_RIGHT = 3;
    public static final int STICK_RIGHT_X = 4;
    public static final int STICK_RIGHT_Y = 5;

    // Joystick Buttons //
    public static final int BTN_A = 1;
    public static final int BTN_B = 2;
    public static final int BTN_X = 3;
    public static final int BTN_Y = 4;
    public static final int BUMPER_LEFT = 5;
    public static final int BUMPER_RIGHT = 6;
    public static final int BTN_BACK = 7;
    public static final int BTN_START = 8;
    public static final int BTN_STICK_LEFT = 9;
    public static final int BTN_STICK_RIGHT = 10;

    // Robot Measurement Constants //
    public static final double TRACK_WIDTH_METERS = 0.555;
    public static final double WHEEL_DIAMETER_METERS = 0.1524;
    public static final double GEAR_RATIO = 10.71;
    public static final double WHEEL_RADIUS_METERS = WHEEL_DIAMETER_METERS / 2;
    public static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;

    // Encoder Constants //
    public static final int ENCODER_CPR = 2048;
    public static final double ENCODER_DISTANTCE_PER_PULSE = 
        (WHEEL_DIAMETER_METERS * Math.PI) / ((double) ENCODER_CPR * GEAR_RATIO);

    // Differential Drive Kinematics //
    public static final DifferentialDriveKinematics DIFF_DRIVE_KINEMATICS =
        new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

    // Feedforward Feedback Gain //
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;

    // DifferentialDriveKinematics //
    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    // Max Trajectory Velocity/Acceleration //
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Ramsete Parameters //
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
