// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dashboard extends SubsystemBase {

  public static String DASH_BASE_ROLL = "Base Pitch:";
  public static String DASH_SHOOTER_POWER = "Shooter Power:";
  public static String DASH_BASE_YAW = "Base Yaw:";
  public static String DASH_BASE_FLPOS = "Base FLPos:";
  public static String DASH_BASE_FRPOS = "Base FRPos:";
  public static String DASH_BASE_BLPOS = "Base BLPos:";
  public static String DASH_BASE_BRPOS = "Base BRPos:";
  public static String DASH_TOWER_BALL_DETECTED = "Tower Ball Detected:";
  public static String DASH_SHOOTER_RPMS = "Shooter RPMs:";
  public static String DASH_SHOOTER_TARGET_RPMS = "Shooter Target RPMs:";
  public static String DASH_SHOOTER_VELOCITY = "Shooter Velocity:";
  public static String DASH_SHOOTER_READY = "Shooter Ready";
  public static String DASH_CLIMBER_LONG_ARM_POSITION = "Climber Long Arm";
  public static String DASH_CLIMBER_SHORT_ARM_POSITION = "Climber Short Arm";
  public static String DASH_HOOD_ANGLE = "Hood Angle";
  public static String DASH_HOOD_VELOCITY = "Hood Velocity";
  public static String DASH_HOOD_POSITION = "Hood Position";
  public static String DASH_HOOD_POSITION_RAW = "Hood Position Raw";
  public static String DASH_HOOD_P = "Hood Kp";
  public static String DASH_HOOD_I = "Hood Ki";
  public static String DASH_HOOD_D = "Hood Kd";
  public static String DASH_HOOD_FF = "Hood ff";
  public static String DASH_HOOD_OUTPUT = "Hood Output";
  public static String DASH_HOOD_DRAW = "Hood Draw";
  public static String DASH_HOOD_TICKS = "Hood Requested Ticks";
  public static String DASH_SHOOTER_P = "Shooter Kp";
  public static String DASH_SHOOTER_FF = "Shooter ff";
  public static String DASH_TARGET_TRACKER_KP = "Target Tracker Kp";
  public static String DASH_TARGET_TRACKER_KI = "Target Tracker Ki";
  public static String DASH_TARGET_TRACKER_KD = "Target Tracker Kd";
  public static String DASH_CLIMBER_LIMITER = "Climber Limiter";

  public static String BASE_NAVIGATE_KP = "Target Tracker Kp";
  public static String BASE_NAVIGATE_KI = "Target Tracker Ki";
  public static String BASE_NAVIGATE_KD = "Target Tracker Kd";

  public static String BASE_ALIGN_NAVIGATE_KP = "Target Tracker Kp";
  public static String BASE_ALIGN_NAVIGATE_KI = "Target Tracker Ki";
  public static String BASE_ALIGN_NAVIGATE_KD = "Target Tracker Kd";

  private Base base;
  private Collector collector;
  private Shooter shooter;
  private Climbers climbers;

  /** Creates a new Dashboard. */
  public Dashboard(
    Base base,
    Collector collector,
    Shooter shooter,
    Climbers climbers
  ) {
    this.base = base;
    this.collector = collector;
    this.shooter = shooter;
    this.climbers = climbers;
  }

  public void initialize() {
    //Values that are init once, user changes for testing or ops, robot reads them when needed
    SmartDashboard.putNumber(DASH_SHOOTER_RPMS, Shooter.SHOOTER_POWER_HUB_HIGH);
  }

  public void publishDashboard() {
    //public information about subsystems
    //this info is read only and put on dashboard for display purposes only
    if (base != null) {
      SmartDashboard.putNumber(DASH_BASE_YAW, this.base.getSensorYaw());
      SmartDashboard.putNumber(
        DASH_BASE_FLPOS,
        this.base.frontLeftModule.getDrivePosition()
      );
      SmartDashboard.putNumber(
        DASH_BASE_FRPOS,
        this.base.frontRightModule.getDrivePosition()
      );
      SmartDashboard.putNumber(
        DASH_BASE_BLPOS,
        this.base.backLeftModule.getDrivePosition()
      );
      SmartDashboard.putNumber(
        DASH_BASE_BRPOS,
        this.base.backRightModule.getDrivePosition()
      );
    }

    if (collector != null) {}
    if (shooter != null) {}
    if (climbers != null) {}
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
