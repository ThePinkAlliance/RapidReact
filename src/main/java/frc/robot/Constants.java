// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;

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
  public static Mk4SwerveModuleHelper.GearRatio motorRatio = Mk4SwerveModuleHelper.GearRatio.L4;

  public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.14528;
  public static final double MAX_ACCELERATION_METERS_PER_SECOND = 6.346;
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = Constants.MAX_VELOCITY_METERS_PER_SECOND
      / Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public static final double ksVolts = 0.22;
  public static final double kvVoltSecondsPerMeter = 1.98;
  public static final double kaVoltSecondsSquaredPerMeter = 0.2;

  // Example value only - as above, this must be tuned for your drive!
  public static final double kPDriveVel = 8.5;

  public static final double kMaxSpeedMetersPerSecond = 3;
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;

  // Reasonable baseline values for a RAMSETE follower in units of meters and
  // seconds
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;

  public static double DRIVETRAIN_WHEELBASE_METERS = 10;
  public static double DRIVETRAIN_TRACKWIDTH_METERS = 5;

  public static double BACK_LEFT_MODULE_STEER_OFFSET = 1;
  public static double BACK_RIGHT_MODULE_STEER_OFFSET = 1;
  public static double FRONT_LEFT_MODULE_STEER_OFFSET = 1;
  public static double FRONT_RIGHT_MODULE_STEER_OFFSET = 1;

  public static int BACK_RIGHT_DRIVE_MOTOR_PORT = 0;
  public static int BACK_LEFT_DRIVE_MOTOR_PORT = 0;
  public static int FRONT_RIGH_DRIVE_MOTOR_PORT = 0;
  public static int FRONT_LEFT_DRIVE_MOTOR_PORT = 0;

  public static int BACK_RIGHT_STEER_MOTOR_PORT = 0;
  public static int BACK_LEFT_STEER_MOTOR_PORT = 0;
  public static int FRONT_RIGHT_STEER_MOTOR_PORT = 0;
  public static int FRONT_LEFT_STEER_MOTOR_PORT = 0;

  public static int BACK_LEFT_CANCODER_ID = 0;
  public static int BACK_RIGHT_CANCODER_ID = 0;
  public static int FRONT_LEFT_CANCODER_ID = 0;
  public static int FRONT_RIGHT_CANCODER_ID = 0;
}
