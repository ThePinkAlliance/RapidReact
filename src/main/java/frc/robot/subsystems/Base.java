// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ThePinkAlliance.swervelib.Mk4ModuleConfiguration;
import com.ThePinkAlliance.swervelib.Mk4iSwerveModuleHelper;
import com.ThePinkAlliance.swervelib.SdsModuleConfigurations;
import com.ThePinkAlliance.swervelib.SwerveModule;
import com.kauailabs.navx.frc.AHRS;
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
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.Supplier;

public class Base extends SubsystemBase {

  public static final double FULL_TALON_ROTATION_TICKS = 2048;

  public static final Mk4iSwerveModuleHelper.GearRatio motorRatio = Mk4iSwerveModuleHelper.GearRatio.L1;

  public static final double MAX_VOLTAGE = 12.0;

  public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(
      23.4);
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(
      23.1);

  public static final double MAX_VELOCITY_METERS_PER_SECOND =
      // 6380 is the theoretical max rpm (e.g. NO LOAD RPM)
      5000.0 /
          60.0 *
          SdsModuleConfigurations.MK4I_L1.getDriveReduction() *
          SdsModuleConfigurations.MK4I_L1.getWheelDiameter() *
          Math.PI; // 5.107;

  public static final double MAX_ACCELERATION_METERS_PER_SECOND = 4.346;
  public static double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
      Math.hypot(
          DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          DRIVETRAIN_WHEELBASE_METERS / 2.0);

  // physical constants
  public static double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(44.82);
  public static double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(226.48); // 179.20
  public static double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(132.71); // 316.66
  public static double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(
      274.65); // 245.97

  public static double circumference = 12.875;

  public static int BACK_RIGHT_DRIVE_MOTOR_PORT = 40;
  public static int BACK_LEFT_DRIVE_MOTOR_PORT = 49;
  public static int FRONT_RIGHT_DRIVE_MOTOR_PORT = 43;
  public static int FRONT_LEFT_DRIVE_MOTOR_PORT = 46;

  public static int BACK_RIGHT_STEER_MOTOR_PORT = 41;
  public static int BACK_LEFT_STEER_MOTOR_PORT = 61;
  public static int FRONT_RIGHT_STEER_MOTOR_PORT = 44;
  public static int FRONT_LEFT_STEER_MOTOR_PORT = 47;

  public static int BACK_LEFT_CANCODER_ID = 62;
  public static int BACK_RIGHT_CANCODER_ID = 42;
  public static int FRONT_RIGHT_CANCODER_ID = 45;
  public static int FRONT_LEFT_CANCODER_ID = 48;

  private final double DRIVE_MOTOR_RAMP_RATE = .10;

  private AHRS gyro = new AHRS(SerialPort.Port.kUSB1);

  private ShuffleboardTab tab;

  /** 0 */
  public final SwerveModule frontLeftModule;

  /** 1 */
  public final SwerveModule frontRightModule;

  /** 2 */
  public final SwerveModule backLeftModule;

  /** 3 */
  public final SwerveModule backRightModule;

  public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      // Front Left Pod
      new Translation2d(
          Base.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          Base.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Front Right
      new Translation2d(
          Base.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          -Base.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back Left
      new Translation2d(
          -Base.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          Base.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back Right
      new Translation2d(
          -Base.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          -Base.DRIVETRAIN_WHEELBASE_METERS / 2.0));

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
      Rotation2d.fromDegrees(gyro.getFusedHeading()));

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
  private SwerveModuleState[] states = kinematics.toSwerveModuleStates(
      chassisSpeeds);

  private Mk4ModuleConfiguration configuration = new Mk4ModuleConfiguration();

  NetworkTableEntry align_P = NetworkTableInstance
      .getDefault()
      .getTable("debug")
      .getEntry("align-p");
  NetworkTableEntry align_I = NetworkTableInstance
      .getDefault()
      .getTable("debug")
      .getEntry("align-i");
  NetworkTableEntry align_D = NetworkTableInstance
      .getDefault()
      .getTable("debug")
      .getEntry("align-d");

  /** Creates a new Base. */
  public Base() {
    this.tab = Shuffleboard.getTab("debug");

    this.backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
        tab
            .getLayout("Back Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0),
        configuration,
        Base.motorRatio,
        Base.BACK_RIGHT_DRIVE_MOTOR_PORT,
        Base.BACK_RIGHT_STEER_MOTOR_PORT,
        Base.BACK_RIGHT_CANCODER_ID,
        Base.BACK_RIGHT_MODULE_STEER_OFFSET);

    this.backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
        tab
            .getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(12, 0),
        configuration,
        Base.motorRatio,
        Base.BACK_LEFT_DRIVE_MOTOR_PORT,
        Base.BACK_LEFT_STEER_MOTOR_PORT,
        Base.BACK_LEFT_CANCODER_ID,
        Base.BACK_LEFT_MODULE_STEER_OFFSET);

    this.frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
        tab
            .getLayout("Front Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(4, 0),
        configuration,
        Base.motorRatio,
        Base.FRONT_RIGHT_DRIVE_MOTOR_PORT,
        Base.FRONT_RIGHT_STEER_MOTOR_PORT,
        Base.FRONT_RIGHT_CANCODER_ID,
        Base.FRONT_RIGHT_MODULE_STEER_OFFSET);

    this.frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
        tab
            .getLayout("Front Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(8, 0),
        configuration,
        Base.motorRatio,
        Base.FRONT_LEFT_DRIVE_MOTOR_PORT,
        Base.FRONT_LEFT_STEER_MOTOR_PORT,
        Base.FRONT_LEFT_CANCODER_ID,
        Base.FRONT_LEFT_MODULE_STEER_OFFSET);

    // Setting the drive motor ramp rate
    this.frontLeftModule.configRampRate(DRIVE_MOTOR_RAMP_RATE);
    this.frontRightModule.configRampRate(DRIVE_MOTOR_RAMP_RATE);
    this.backLeftModule.configRampRate(DRIVE_MOTOR_RAMP_RATE);
    this.backRightModule.configRampRate(DRIVE_MOTOR_RAMP_RATE);
  }

  public void resetDriveMotors() {
    this.backLeftModule.resetDrive();
    this.backRightModule.resetDrive();
    this.frontLeftModule.resetDrive();
    this.frontRightModule.resetDrive();
  }

  public SwerveModuleState[] getStates() {
    return this.states;
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
  public void setStates(SwerveModuleState... states) {
    odometry.update(getRotation(), states);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        states,
        Base.MAX_VELOCITY_METERS_PER_SECOND);

    var translation = odometry.getPoseMeters().getTranslation();
    m_xEntry.setNumber(translation.getX());
    m_yEntry.setNumber(translation.getY());

    this.frontLeftModule.set(
        (states[0].speedMetersPerSecond / Base.MAX_VELOCITY_METERS_PER_SECOND) *
            Base.MAX_VOLTAGE,
        states[0].angle.getRadians());

    this.frontRightModule.set(
        (states[1].speedMetersPerSecond / Base.MAX_VELOCITY_METERS_PER_SECOND) *
            Base.MAX_VOLTAGE,
        states[1].angle.getRadians());

    this.backLeftModule.set(
        (states[2].speedMetersPerSecond / Base.MAX_VELOCITY_METERS_PER_SECOND) *
            Base.MAX_VOLTAGE,
        states[2].angle.getRadians());

    this.backRightModule.set(
        (states[3].speedMetersPerSecond / Base.MAX_VELOCITY_METERS_PER_SECOND) *
            Base.MAX_VOLTAGE,
        states[3].angle.getRadians());
  }

  public double getRoll() {
    return gyro.getRoll();
  }

  public double getPitch() {
    return gyro.getPitch();
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

  public AHRS getGyro() {
    return this.gyro;
  }

  public double getSensorYaw() {
    return gyro.getYaw();
  }

  /**
   * Returns if the robot inverted.
   */
  public boolean isInverted() {
    return (getRotation().getDegrees() <= 190 &&
        getRotation().getDegrees() > 90 ||
        getRotation().getDegrees() >= 290 &&
            getRotation().getDegrees() < 90);
  }

  /**
   * Returns the current odometry pose of the robot.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns the current odometry pose of the robot as a supplier.
   */
  public Supplier<Pose2d> getPoseSupplier() {
    return () -> odometry.getPoseMeters();
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_yaw.setNumber(gyro.getYaw());

    Constants.isRed = NetworkTableInstance
        .getDefault()
        .getTable("FMSInfo")
        .getEntry("IsRedAlliance")
        .getBoolean(true);

    setStates(this.states);
    SmartDashboard.putNumber(Dashboard.DASH_BASE_ROLL, gyro.getRoll());
  }

  @Override
  public void simulationPeriodic() {
    setStates(this.states);
  }
}
