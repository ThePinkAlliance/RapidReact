// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ThePinkAlliance.swervelib.Mk4ModuleConfiguration;
import com.ThePinkAlliance.swervelib.Mk4SwerveModuleHelper;
import com.ThePinkAlliance.swervelib.SdsModuleConfigurations;
import com.ThePinkAlliance.swervelib.SwerveModule;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Base extends SubsystemBase {

  public static double TALON_ROTATION_TICKS = 2048;

  public static final Mk4SwerveModuleHelper.GearRatio motorRatio =
    Mk4SwerveModuleHelper.GearRatio.L1;

  public static final double MAX_VOLTAGE = 12.0;

  public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(
    23.4
  );
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(
    23.1
  );

  public static final double MAX_VELOCITY_METERS_PER_SECOND =
    //6380 is the theoretical max rpm (e.g. NO LOAD RPM)
    //TODO - select a realistic rpm.
    5000.0 /
    60.0 *
    SdsModuleConfigurations.MK4_L1.getDriveReduction() *
    SdsModuleConfigurations.MK4_L1.getWheelDiameter() *
    Math.PI; // 5.107;

  public static final double MAX_ACCELERATION_METERS_PER_SECOND = 4.346;
  public static double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
    MAX_VELOCITY_METERS_PER_SECOND /
    Math.hypot(
      DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
      DRIVETRAIN_WHEELBASE_METERS / 2.0
    );

  // physical constants
  public static double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(188.69);
  public static double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(182.97); // 179.20
  public static double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(132.09);
  public static double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(63.80); // 359.29

  public static double circumference =
    Units.metersToInches(SdsModuleConfigurations.MK4_L1.getWheelDiameter()) *
    Math.PI;

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

  private double previous_x = 0;
  private double previous_y = 0;

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

  NetworkTableEntry m_xEntry = NetworkTableInstance
    .getDefault()
    .getTable("troubleshooting")
    .getEntry("X");

  NetworkTableEntry m_yEntry = NetworkTableInstance
    .getDefault()
    .getTable("troubleshooting")
    .getEntry("Y");

  NetworkTableEntry m_rotEntry = NetworkTableInstance
    .getDefault()
    .getTable("troubleshooting")
    .getEntry("rot");

  NetworkTableEntry m_yaw = NetworkTableInstance
    .getDefault()
    .getTable("debug")
    .getEntry("yaw");

  NetworkTableEntry m_yaw_diff = NetworkTableInstance
    .getDefault()
    .getTable("debug")
    .getEntry("yaw_diff");

  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
    kinematics,
    Rotation2d.fromDegrees(gyro.getFusedHeading())
  );

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
  private SwerveModuleState[] states = kinematics.toSwerveModuleStates(
    chassisSpeeds
  );

  private Mk4ModuleConfiguration configuration = new Mk4ModuleConfiguration();

  private ChassisSpeeds autoSpeeds = new ChassisSpeeds();

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
  }

  public void resetDriveMotors() {
    this.backLeftModule.resetDrive();
    this.backRightModule.resetDrive();
    this.frontLeftModule.resetDrive();
    this.frontRightModule.resetDrive();
  }

  /**
   * NOTE: This needs to use the gyro to keep it from drifting from the desired heading
   *
   * @param targetPosition the desired unit of measurement is inches
   * @return
   */
  public boolean driveStraight(double targetPosition, double targetAngle) {
    if (targetAngle == 0) {
      targetAngle = 360;
    }

    double front_left_pos = Math.abs(this.frontLeftModule.getDrivePosition());
    double front_right_pos = Math.abs(this.frontRightModule.getDrivePosition());
    Rotation2d angle_diff = getRotation()
      .minus(Rotation2d.fromDegrees(targetAngle));

    if (angle_diff.getDegrees() < -0) {
      angle_diff = Rotation2d.fromDegrees((360 - targetAngle));
    }

    double front_left_rot =
      SdsModuleConfigurations.MK4_L1.getDriveReduction() *
      (front_left_pos / 2048);

    double front_right_rot =
      SdsModuleConfigurations.MK4_L1.getDriveReduction() *
      (front_right_pos / 2048);

    double front_left_inches = front_left_rot * circumference;
    double front_right_inches = front_right_rot * circumference;

    double distance_traveled_inches =
      (front_left_inches + front_right_inches) / 2.0;

    if (distance_traveled_inches >= targetPosition) {
      autoSpeeds = new ChassisSpeeds(0, 0, 0);
      // previous_x = distance_traveled_inches;
    } else if (distance_traveled_inches < targetPosition) {
      autoSpeeds = new ChassisSpeeds(1, 0, 0);
      setPodAngles(angle_diff);
    }

    m_yaw_diff.setNumber(angle_diff.getDegrees());

    SmartDashboard.putNumber("front_left_inches", front_left_inches);
    SmartDashboard.putNumber("front_right_inches", front_right_inches);
    SmartDashboard.putNumber(
      "distance_traveled_inches",
      distance_traveled_inches
    );
    SmartDashboard.putNumber("front_left_rot", front_left_rot);
    SmartDashboard.putNumber("front_right_rot", front_right_rot);

    setStates(kinematics.toSwerveModuleStates(autoSpeeds));

    return distance_traveled_inches >= targetPosition;
  }

  public void setPodAngles(double angle) {
    for (int i = 0; i < states.length; i++) {
      SwerveModuleState state = states[i];

      state.angle = new Rotation2d(angle);

      states[i] = state;
    }
  }

  public void setPodAngles(Rotation2d angle) {
    for (int i = 0; i < states.length; i++) {
      SwerveModuleState state = states[i];

      state.angle = angle;

      states[i] = state;
    }
  }

  public boolean rotate(double target) {
    double heading = getRotation().getDegrees();
    double error = 5;

    double power = getPower(target);

    m_rotEntry.setNumber(heading);

    if (heading >= (target - error) && heading <= (target + error)) {
      setStates(kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
      return true;
    } else {
      setStates(
        kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, power))
      );
      return false;
    }
  }

  /**
   * NOTE: This needs a better implementation of finding the shortest direction to travel
   * @param angle
   * @param target
   * @return a positive or negiative power in meters per second.
   */
  private double getPower(double target) {
    double sign = 1;
    double heading = getRotation().getDegrees();

    if (target > heading) {
      sign = 1;
    } else if (target < heading) {
      sign = -1;
    }

    return Math.copySign(1.4, sign);
  }

  /**
   * This will calulate the gear reduction for the encoder.
   * @param ticks This will be the ticks from the motor (Raw encoder units)
   */
  // public double calulateWheelTicks(double ticks) {
  //   return (((ticks / 1.30) / 1.13) * 1.1) / FULL_ROT_TICKS;
  // }

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
    odometry.update(getRotation(), states);

    SwerveDriveKinematics.desaturateWheelSpeeds(
      states,
      Base.MAX_VELOCITY_METERS_PER_SECOND
    );

    var translation = odometry.getPoseMeters().getTranslation();
    m_xEntry.setNumber(translation.getX());
    m_yEntry.setNumber(translation.getY());

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
   * This resets the odometry to the given position and sets the rotation to the
   * current one from the gyro.
   */
  public void resetOdometry(Pose2d pose, Rotation2d rot) {
    this.odometry.resetPosition(pose, rot);
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

    m_yaw.setNumber(gyro.getFusedHeading());

    setStates(this.states);
  }

  @Override
  public void simulationPeriodic() {
    setStates(this.states);
  }
}
