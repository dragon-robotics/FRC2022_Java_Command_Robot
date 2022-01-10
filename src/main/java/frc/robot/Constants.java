// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    final static int DRIVER = 0;
    final static int OPERATOR = 1;

    // Joystick Mapping IDs //

    // Joystick Analog Axis/Stick //
    final static int STICK_LEFT_X = 0;
    final static int STICK_LEFT_Y = 1;
    final static int TRIGGER_LEFT = 2;
    final static int TRIGGER_RIGHT = 3;
    final static int STICK_RIGHT_X = 4;
    final static int STICK_RIGHT_Y = 5;

    // Joystick Buttons //
    final static int BTN_A = 1;
    final static int BTN_B = 2;
    final static int BTN_X = 3;
    final static int BTN_Y = 4;
    final static int BUMPER_LEFT = 5;
    final static int BUMPER_RIGHT = 6;
    final static int BTN_BACK = 7;
    final static int BTN_START = 8;
    final static int BTN_STICK_LEFT = 9;
    final static int BTN_STICK_RIGHT = 10;
}
