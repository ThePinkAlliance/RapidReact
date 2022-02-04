package frc.robot.constants;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;

public class Base {
  public final Mk4SwerveModuleHelper.GearRatio motorRatio = Mk4SwerveModuleHelper.GearRatio.L4;

  public final double DRIVETRAIN_WHEELBASE_METERS = 24;
  public final double DRIVETRAIN_TRACKWIDTH_METERS = 30;

  public final double MAX_VELOCITY_METERS_PER_SECOND = 13.14528;
  public final double MAX_ACCELERATION_METERS_PER_SECOND = 6.346;
  public double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
      / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public double kMaxSpeedMetersPerSecond = 3;
  public double kMaxAccelerationMetersPerSecondSquared = 3;

  // Reasonable baseline values for a RAMSETE follower in units of meters and
  // seconds
  public double kRamseteB = 2;
  public double kRamseteZeta = 0.7;

  public double BACK_LEFT_MODULE_STEER_OFFSET = 1;
  public double BACK_RIGHT_MODULE_STEER_OFFSET = 1;
  public double FRONT_LEFT_MODULE_STEER_OFFSET = 1;
  public double FRONT_RIGHT_MODULE_STEER_OFFSET = 1;

  public int BACK_RIGHT_DRIVE_MOTOR_PORT = 0;
  public int BACK_LEFT_DRIVE_MOTOR_PORT = 0;
  public int FRONT_RIGH_DRIVE_MOTOR_PORT = 0;
  public int FRONT_LEFT_DRIVE_MOTOR_PORT = 20;

  public int BACK_RIGHT_STEER_MOTOR_PORT = 0;
  public int BACK_LEFT_STEER_MOTOR_PORT = 0;
  public int FRONT_RIGHT_STEER_MOTOR_PORT = 0;
  public int FRONT_LEFT_STEER_MOTOR_PORT = 21;

  public int BACK_LEFT_CANCODER_ID = 0;
  public int BACK_RIGHT_CANCODER_ID = 0;
  public int FRONT_LEFT_CANCODER_ID = 0;
  public int FRONT_RIGHT_CANCODER_ID = 0;
}