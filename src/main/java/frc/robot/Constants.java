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

  //Motor constants
  public static final int LEFT_FRONT_MOTOR = 0;
  public static final int LEFT_REAR_MOTOR = 1;
  public static final int RIGHT_FRONT_MOTOR = 2;
  public static final int RIGHT_REAR_MOTOR = 3;

  //Controller constants

  //controller - Driver
  public static final int DRIVER_JOYSTICK = 0;

  //controller - Driver Axis maps
  public static final int DRIVER_JOYSTICK_LEFT_X_AXIS = 0;
  public static final int DRIVER_JOYSTICK_LEFT_Y_AXIS = 1;
  public static final int DRIVER_JOYSTICK_RIGHT_X_AXIS = 2;
  public static final int DRIVER_JOYSTICK_RIGHT_Y_AXIS = 3;

  //controller - Driver Button maps
  public static final int DRIVER_LEFT = 1;
  public static final int DRIVER_RIGHT = 3;
  public static final int DRIVER_UP = 4;
  public static final int DRIVER_DOWN = 2;
  public static final int DRIVER_SHOULDER_TOP_LEFT = 5;
  public static final int DRIVER_SHOULDER_TOP_RIGHT = 6;
  public static final int DRIVER_SHOULDER_BOTTOM_LEFT = 7;
  public static final int DRIVER_SHOULDER_BOTTOM_RIGHT = 8;
  public static final int DRIVER_LEFT_JOYSTICK = 9;
  public static final int DRIVER_RIGHT_JOYSTICK = 10;

  //drivetrain speed constants
  public static final double DRIVETRAIN_SPEED = 1;

  //controller - operator
  public static final int OPERATOR_JOYSTICK = 1;

  //controller - Operator Axis maps
  public static final int OPERATOR_JOYSTICK_LEFT_X_AXIS = 0;
  public static final int OPERATOR_JOYSTICK_LEFT_Y_AXIS = 1;
  public static final int OPERATOR_JOYSTICK_RIGHT_X_AXIS = 2;
  public static final int OPERATOR_JOYSTICK_RIGHT_Y_AXIS = 3;

  //controller - Driver Button maps
  public static final int OPERATOR_LEFT = 1;
  public static final int OPERATOR_RIGHT = 3;
  public static final int OPERATOR_UP = 4;
  public static final int OPERATOR_DOWN = 2;
  public static final int OPERATOR_SHOULDER_TOP_LEFT = 5;
  public static final int OPERATOR_SHOULDER_TOP_RIGHT = 6;
  public static final int OPERATOR_SHOULDER_BOTTOM_LEFT = 7;
  public static final int OPERATOR_SHOULDER_BOTTOM_RIGHT = 8;
  public static final int OPERATOR_LEFT_JOYSTICK = 9;
  public static final int OPERATOR_RIGHT_JOYSTICK = 10;
}