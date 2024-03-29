// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.drive.Vector2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {

  public static boolean isRed = true;

  public static int JOYSTICK_BUTTON_X = 3;
  public static int JOYSTICK_BUTTON_A = 1;
  public static int JOYSTICK_BUTTON_B = 2;
  public static int JOYSTICK_BUTTON_Y = 4;
  public static int JOYSTICK_LEFT_Y_AXIS_BUTTON = 9;
  public static int JOYSTICK_LEFT_Y_AXIS = 1;
  public static int JOYSTICK_RIGHT_Y_AXIS = 5;

  public static int JOYSTICK_POV_UP = 0;
  public static int JOYSTICK_POV_RIGHT = 90;
  public static int JOYSTICK_POV_DOWN = 180;
  public static int JOYSTICK_POV_LEFT = 270;

  public static int JOYSTICK_LEFT_TRIGGER = 2;
  public static int JOYSTICK_RIGHT_TRIGGER = 3;

  public static int JOYSTICK_LEFT_BUMPER = 5;
  public static int JOYSTICK_RIGHT_BUMPER = 6;

  // (ty-angle, distance)
  static List<Vector2d> distanceTable = List.of(
      new Vector2d(46, 95),
      new Vector2d(54, 101),
      new Vector2d(64, 109),
      new Vector2d(77, 119),
      new Vector2d(90, 129),
      new Vector2d(
          101, 140),
      new Vector2d(
          112, 149));

  public static LinearInterpolationTable limelightInterpolationTable = new LinearInterpolationTable(distanceTable);
}
