// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Map;

public class Base extends SubsystemBase {

  private AHRS gyro = new AHRS();

  private ShuffleboardTab tab;

  /** 0 */
  private final SwerveModule frontLeftModule;

  /** 1 */
  private final SwerveModule frontRightModule;

  /** 2 */
  private final SwerveModule backLeftModule;

  /** 3 */
  private final SwerveModule backRightModule;

  public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    // Front Left Pod
    new Translation2d(
      Constants.Base.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
      Constants.Base.DRIVETRAIN_WHEELBASE_METERS / 2.0
    ),
    // Front Right
    new Translation2d(
      Constants.Base.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
      -Constants.Base.DRIVETRAIN_WHEELBASE_METERS / 2.0
    ),
    // Back Left
    new Translation2d(
      -Constants.Base.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
      Constants.Base.DRIVETRAIN_WHEELBASE_METERS / 2.0
    ),
    // Back Right
    new Translation2d(
      -Constants.Base.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
      -Constants.Base.DRIVETRAIN_WHEELBASE_METERS / 2.0
    )
  );

  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
    kinematics,
    Rotation2d.fromDegrees(gyro.getFusedHeading())
  );

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
  private SwerveModuleState[] states = kinematics.toSwerveModuleStates(
    chassisSpeeds
  );

  private Mk4ModuleConfiguration configuration = new Mk4ModuleConfiguration();

  /** Creates a new Base. */
  public Base() {
    this.tab = Shuffleboard.getTab("debug");

    this.backRightModule =
      Mk4SwerveModuleHelper.createFalcon500(
        tab
          .getLayout("Back Right Module", BuiltInLayouts.kList)
          .withSize(2, 4)
          .withPosition(0, 0),
        configuration,
        Constants.Base.motorRatio,
        Constants.Base.BACK_RIGHT_DRIVE_MOTOR_PORT,
        Constants.Base.BACK_RIGHT_STEER_MOTOR_PORT,
        Constants.Base.BACK_RIGHT_CANCODER_ID,
        Constants.Base.BACK_RIGHT_MODULE_STEER_OFFSET
      );

    this.backLeftModule =
      Mk4SwerveModuleHelper.createFalcon500(
        tab
          .getLayout("Back Left Module", BuiltInLayouts.kList)
          .withSize(2, 4)
          .withPosition(12, 0),
        configuration,
        Constants.Base.motorRatio,
        Constants.Base.BACK_LEFT_DRIVE_MOTOR_PORT,
        Constants.Base.BACK_LEFT_STEER_MOTOR_PORT,
        Constants.Base.BACK_LEFT_CANCODER_ID,
        Constants.Base.BACK_LEFT_MODULE_STEER_OFFSET
      );

    this.frontRightModule =
      Mk4SwerveModuleHelper.createFalcon500(
        tab
          .getLayout("Front Right Module", BuiltInLayouts.kList)
          .withSize(2, 4)
          .withPosition(4, 0),
        configuration,
        Constants.Base.motorRatio,
        Constants.Base.FRONT_RIGHT_DRIVE_MOTOR_PORT,
        Constants.Base.FRONT_RIGHT_STEER_MOTOR_PORT,
        Constants.Base.FRONT_RIGHT_CANCODER_ID,
        Constants.Base.FRONT_RIGHT_MODULE_STEER_OFFSET
      );

    this.frontLeftModule =
      Mk4SwerveModuleHelper.createFalcon500(
        tab
          .getLayout("Front Left Module", BuiltInLayouts.kList)
          .withSize(2, 4)
          .withPosition(8, 0),
        configuration,
        Constants.Base.motorRatio,
        Constants.Base.FRONT_LEFT_DRIVE_MOTOR_PORT,
        Constants.Base.FRONT_LEFT_STEER_MOTOR_PORT,
        Constants.Base.FRONT_LEFT_CANCODER_ID,
        Constants.Base.FRONT_LEFT_MODULE_STEER_OFFSET
      );
  }

  /**
   * Sets the current chassis speeds of the robot to the given speeds and updates
   * the swerve module states to the current robot speeds.
   */
  public void drive(ChassisSpeeds speeds) {
    this.chassisSpeeds = speeds;

    this.states = kinematics.toSwerveModuleStates(chassisSpeeds);
  }

  /**
   * Set the robot's states to the given states.
   *
   * @deprecated DO NOT USE THIS METHOD ITS NOT UPDATED.
   */
  @Deprecated
  public void setStates(SwerveModuleState[] states) {
    this.frontLeftModule.set(
        (
          states[0].speedMetersPerSecond /
          Constants.Base.MAX_VELOCITY_METERS_PER_SECOND
        ) *
        1.0,
        states[0].angle.getRadians()
      );
    this.frontRightModule.set(
        (
          states[1].speedMetersPerSecond /
          Constants.Base.MAX_VELOCITY_METERS_PER_SECOND
        ) *
        -1.0,
        states[1].angle.getRadians()
      );
    this.backLeftModule.set(
        (
          states[2].speedMetersPerSecond /
          Constants.Base.MAX_VELOCITY_METERS_PER_SECOND
        ) *
        1.0,
        states[2].angle.getRadians()
      );
    this.backRightModule.set(
        (
          states[3].speedMetersPerSecond /
          Constants.Base.MAX_VELOCITY_METERS_PER_SECOND
        ) *
        -1.0,
        states[3].angle.getRadians()
      );

    odometry.update(getRotation(), this.states);
  }

  /**
   * @return ChassisSpeeds of the robot
   */
  public ChassisSpeeds getChassisSpeeds() {
    return chassisSpeeds;
  }

  /**
   * This resets the gyroscope's Yaw axis to zero.
   */
  public void zeroGyro() {
    gyro.reset();
  }

  /**
   * This resets the odometry to the given position and sets the rotation to the
   * current one from the gyro.
   */
  public void resetOdometry(Pose2d pose) {
    this.odometry.resetPosition(pose, this.getRotation());
  }

  /**
   * Returns the robot's current rotation.
   *
   * @return the robot's current rotation.
   */
  public Rotation2d getRotation() {
    if (gyro.isMagnetometerCalibrated()) {
      return Rotation2d.fromDegrees(gyro.getFusedHeading());
    }

    return Rotation2d.fromDegrees(360.0 - gyro.getYaw());
  }

  /**
   * Returns if the robot inverted.
   */
  public boolean isInverted() {
    return (
      getRotation().getDegrees() <= 190 &&
      getRotation().getDegrees() > 90 ||
      getRotation().getDegrees() >= 290 &&
      getRotation().getDegrees() < 90
    );
  }

  /**
   * Returns the current odometry pose of the robot.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns the current direction of the robot.
   */
  public double getDirection() {
    if (isInverted()) {
      return 1.0;
    }

    return -1.0;
  }

  public double targetAngleToPower(
    double angle,
    double power,
    double forwardAngle,
    double strafeAngle
  ) {
    double angleDeg = Math.toDegrees(angle);
    double forwardDeg = Math.toDegrees(forwardAngle);
    double strafeDeg = Math.toDegrees(strafeAngle);

    double rotHypot = Math.hypot(strafeDeg, forwardDeg);
    double omega = chassisSpeeds.omegaRadiansPerSecond;

    // TODO: replace this with the hypotenuse of the angle: done.
    if (Math.abs(angleDeg) == Math.abs(rotHypot)) {
      return power * omega;
    }

    return power;
  }

  public double calulateStrafe(double offset) {
    return (offset - 360) / 2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    odometry.update(getRotation(), this.states);

    SwerveDriveKinematics.desaturateWheelSpeeds(
      states,
      Constants.Base.MAX_VELOCITY_METERS_PER_SECOND
    );

    /**
     * Inverting the requested angle when its in rotation position to allow to the robot to turn?
     */
    double frontLeftPower = targetAngleToPower(
      (
        states[0].speedMetersPerSecond /
        Constants.Base.MAX_VELOCITY_METERS_PER_SECOND
      ) *
      Constants.Base.MAX_VOLTAGE,
      states[0].angle.getRadians(),
      Constants.Base.FRONT_LEFT_MODULE_STEER_OFFSET,
      calulateStrafe(Constants.Base.FRONT_LEFT_MODULE_STEER_OFFSET)
    );

    this.frontLeftModule.set(frontLeftPower, states[0].angle.getRadians());

    double frontRightPower = targetAngleToPower(
      (
        states[1].speedMetersPerSecond /
        Constants.Base.MAX_VELOCITY_METERS_PER_SECOND
      ) *
      Constants.Base.MAX_VOLTAGE,
      states[1].angle.getRadians(),
      Constants.Base.FRONT_RIGHT_MODULE_STEER_OFFSET,
      calulateStrafe(Constants.Base.FRONT_RIGHT_MODULE_STEER_OFFSET)
    );

    this.frontRightModule.set(frontRightPower, states[1].angle.getRadians());

    double backLeftPower = targetAngleToPower(
      (
        states[2].speedMetersPerSecond /
        Constants.Base.MAX_VELOCITY_METERS_PER_SECOND
      ) *
      Constants.Base.MAX_VOLTAGE,
      states[2].angle.getRadians(),
      Constants.Base.BACK_LEFT_MODULE_STEER_OFFSET,
      calulateStrafe(Constants.Base.BACK_LEFT_MODULE_STEER_OFFSET)
    );

    this.backLeftModule.set(backLeftPower, states[2].angle.getRadians());

    double backRightPower = targetAngleToPower(
      (
        states[3].speedMetersPerSecond /
        Constants.Base.MAX_VELOCITY_METERS_PER_SECOND
      ) *
      Constants.Base.MAX_VOLTAGE,
      states[3].angle.getRadians(),
      Constants.Base.BACK_RIGHT_MODULE_STEER_OFFSET,
      calulateStrafe(Constants.Base.BACK_RIGHT_MODULE_STEER_OFFSET)
    );

    this.backRightModule.set(backRightPower, states[3].angle.getRadians());
  }
}
