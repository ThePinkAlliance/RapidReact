// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Base extends SubsystemBase {

  public static final Mk4SwerveModuleHelper.GearRatio motorRatio =
    Mk4SwerveModuleHelper.GearRatio.L4;

  public static final double MAX_VOLTAGE = 12.0;

  public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(
    24
  );
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(
    24
  );

  public static final double MAX_VELOCITY_METERS_PER_SECOND =
    //6380 is the theoretical max rpm (e.g. NO LOAD RPM)
    //TODO - select a realistic rpm.
    5000.0 /
    60.0 *
    SdsModuleConfigurations.MK4_L4.getDriveReduction() *
    SdsModuleConfigurations.MK4_L4.getWheelDiameter() *
    Math.PI; // 13.14528;

  public static final double MAX_ACCELERATION_METERS_PER_SECOND = 6.346;
  public static double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
    MAX_VELOCITY_METERS_PER_SECOND /
    Math.hypot(
      DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
      DRIVETRAIN_WHEELBASE_METERS / 2.0
    );

  // physical constants
  public static double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(188.69);
  public static double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(179.20);
  public static double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(132.09);
  public static double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(63.80); // 359.29

  // Simulated constants
  // public static double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0);
  // public static double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0);
  // public static double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0);
  // public static double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0); // 359.29

  public static int BACK_RIGHT_DRIVE_MOTOR_PORT = 27;
  public static int BACK_LEFT_DRIVE_MOTOR_PORT = 25;
  public static int FRONT_RIGHT_DRIVE_MOTOR_PORT = 21;
  public static int FRONT_LEFT_DRIVE_MOTOR_PORT = 23;

  public static int BACK_RIGHT_STEER_MOTOR_PORT = 26;
  public static int BACK_LEFT_STEER_MOTOR_PORT = 24;
  public static int FRONT_RIGHT_STEER_MOTOR_PORT = 20;
  public static int FRONT_LEFT_STEER_MOTOR_PORT = 22;

  public static int BACK_LEFT_CANCODER_ID = 32;
  public static int BACK_RIGHT_CANCODER_ID = 33;
  public static int FRONT_LEFT_CANCODER_ID = 31;
  public static int FRONT_RIGHT_CANCODER_ID = 34;

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
      Base.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
      Base.DRIVETRAIN_WHEELBASE_METERS / 2.0
    ),
    // Front Right
    new Translation2d(
      Base.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
      -Base.DRIVETRAIN_WHEELBASE_METERS / 2.0
    ),
    // Back Left
    new Translation2d(
      -Base.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
      Base.DRIVETRAIN_WHEELBASE_METERS / 2.0
    ),
    // Back Right
    new Translation2d(
      -Base.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
      -Base.DRIVETRAIN_WHEELBASE_METERS / 2.0
    )
  );

  private PIDController driveController = new PIDController(1, 0, 0);

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
        Base.motorRatio,
        Base.BACK_RIGHT_DRIVE_MOTOR_PORT,
        Base.BACK_RIGHT_STEER_MOTOR_PORT,
        Base.BACK_RIGHT_CANCODER_ID,
        Base.BACK_RIGHT_MODULE_STEER_OFFSET
      );

    this.backLeftModule =
      Mk4SwerveModuleHelper.createFalcon500(
        tab
          .getLayout("Back Left Module", BuiltInLayouts.kList)
          .withSize(2, 4)
          .withPosition(12, 0),
        configuration,
        Base.motorRatio,
        Base.BACK_LEFT_DRIVE_MOTOR_PORT,
        Base.BACK_LEFT_STEER_MOTOR_PORT,
        Base.BACK_LEFT_CANCODER_ID,
        Base.BACK_LEFT_MODULE_STEER_OFFSET
      );

    this.frontRightModule =
      Mk4SwerveModuleHelper.createFalcon500(
        tab
          .getLayout("Front Right Module", BuiltInLayouts.kList)
          .withSize(2, 4)
          .withPosition(4, 0),
        configuration,
        Base.motorRatio,
        Base.FRONT_RIGHT_DRIVE_MOTOR_PORT,
        Base.FRONT_RIGHT_STEER_MOTOR_PORT,
        Base.FRONT_RIGHT_CANCODER_ID,
        Base.FRONT_RIGHT_MODULE_STEER_OFFSET
      );

    this.frontLeftModule =
      Mk4SwerveModuleHelper.createFalcon500(
        tab
          .getLayout("Front Left Module", BuiltInLayouts.kList)
          .withSize(2, 4)
          .withPosition(8, 0),
        configuration,
        Base.motorRatio,
        Base.FRONT_LEFT_DRIVE_MOTOR_PORT,
        Base.FRONT_LEFT_STEER_MOTOR_PORT,
        Base.FRONT_LEFT_CANCODER_ID,
        Base.FRONT_LEFT_MODULE_STEER_OFFSET
      );

    configuration.setDriveCurrentLimit(50);
  }

  public void resetDriveMotors() {
    this.backLeftModule.resetDrive();
    this.backRightModule.resetDrive();
    this.frontLeftModule.resetDrive();
    this.frontRightModule.resetDrive();
  }

  public boolean driveStraight(double targetPosition) {
    double frontLeftPos = this.frontLeftModule.getDrivePosition();
    double frontRightPos = this.frontRightModule.getDrivePosition();
    double backLeftPos = this.backLeftModule.getDrivePosition();
    double backRightPos = this.backRightModule.getDrivePosition();

    double circumference =
      SdsModuleConfigurations.MK4_L4.getWheelDiameter() * Math.PI;

    double rot_ticks =
      SdsModuleConfigurations.MK4_L4.getDriveReduction() *
      Constants.TALON_ROTATION_TICKS;

    return false;
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
   */
  public void setStates(SwerveModuleState[] states) {
    odometry.update(getRotation(), this.states);

    SwerveDriveKinematics.desaturateWheelSpeeds(
      states,
      Base.MAX_VELOCITY_METERS_PER_SECOND
    );

    this.frontLeftModule.set(
        (states[0].speedMetersPerSecond / Base.MAX_VELOCITY_METERS_PER_SECOND) *
        Base.MAX_VOLTAGE,
        states[0].angle.getRadians()
      );

    this.frontRightModule.set(
        (states[1].speedMetersPerSecond / Base.MAX_VELOCITY_METERS_PER_SECOND) *
        Base.MAX_VOLTAGE,
        states[1].angle.getRadians()
      );

    this.backLeftModule.set(
        (states[2].speedMetersPerSecond / Base.MAX_VELOCITY_METERS_PER_SECOND) *
        Base.MAX_VOLTAGE,
        states[2].angle.getRadians()
      );

    this.backRightModule.set(
        (states[3].speedMetersPerSecond / Base.MAX_VELOCITY_METERS_PER_SECOND) *
        Base.MAX_VOLTAGE,
        states[3].angle.getRadians()
      );
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

  public boolean isWithinError(double target, double current) {
    return (target - current) < 2;
  }

  public boolean isOppsite(double v1, double v2) {
    return (v1 > -0 && v2 < 0) || (v1 > 0 && v2 > -0);
  }

  @Deprecated
  public double calulateStrafe(double offset) {
    return (360 - offset) / 2;
  }

  @Deprecated
  public double invertPower(double invertAngle, double angle, double power) {
    boolean in_range = angle <= (invertAngle + 2) && angle <= (invertAngle - 2);

    if (in_range) {
      return power * -1.0;
    }

    return power;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    setStates(this.states);
  }

  @Override
  public void simulationPeriodic() {
    setStates(this.states);
  }
}
