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

  public double BACK_LEFT_MODULE_STEER_OFFSET = 1;
  public double BACK_RIGHT_MODULE_STEER_OFFSET = 1;
  public double FRONT_LEFT_MODULE_STEER_OFFSET = 1;
  public double FRONT_RIGHT_MODULE_STEER_OFFSET = 1;

  public int BACK_RIGHT_DRIVE_MOTOR_PORT = 27;
  public int BACK_LEFT_DRIVE_MOTOR_PORT = 25;
  public int FRONT_RIGHT_DRIVE_MOTOR_PORT = 23;
  public int FRONT_LEFT_DRIVE_MOTOR_PORT = 21;

  public int BACK_RIGHT_STEER_MOTOR_PORT = 26;
  public int BACK_LEFT_STEER_MOTOR_PORT = 24;
  public int FRONT_RIGHT_STEER_MOTOR_PORT = 20;
  public int FRONT_LEFT_STEER_MOTOR_PORT = 22;

  public int BACK_LEFT_CANCODER_ID = 22;
  public int BACK_RIGHT_CANCODER_ID = 23;
  public int FRONT_LEFT_CANCODER_ID = 21;
  public int FRONT_RIGHT_CANCODER_ID = 24;
}