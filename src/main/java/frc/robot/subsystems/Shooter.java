// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ThePinkAlliance.core.math.Projectile;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShooterConstants;

public class Shooter extends SubsystemBase {

  public static final double SHOOTER_POWER_HUB_HIGH = 2450;
  public static final double SHOOTER_POWER_TARMAC_HIGH = 2730;
  public static final double SHOOTER_POWER_THREE_BALL = 2730;
  public static final double SHOOTER_POWER_AUTO_TWO_BALL = 2750;
  public static final double SHOOTER_Kp_AUTO_TWO_BALL = 0.4;
  public static final double SHOOTER_FF_AUTO_TWO_BALL = 0.047;
  public static final double SHOOTER_POWER_HUB_LOW = 1800;

  public static final double SHOOTER_Kp_AUTO_THREE_BALL = 0.4;
  public static final double SHOOTER_FF_AUTO_THREE_BALL = 0.047;

  public double CURRENT_HOOD_ANGLE = 45;

  public static final double SHOOTER_FLYWHEEL_DIAMETER = 4.063;
  public static final double SHOOTER_FLYWHEEL_CIRCUMFRENCE =
    (SHOOTER_FLYWHEEL_DIAMETER / 2) * Math.PI;

  private double RAMP_RATE = 0;
  private double NOMINAL_FORWARD = 0;
  private double NOMINAL_REVERSE = 0;
  public static double GEAR_MULTIPLYER = 1.6;
  private double PEAK_FORWARD = 1;
  private double PEAK_REVERSE = -1;
  private final int SHOOTER_MOTOR = 30;
  private boolean isActivated = false;

  public static final double CARGO_INCOMMING_ANGLE = -69;

  // remeasure this
  public static final double SHOOTER_FROM_GROUND = 23.27;

  private TalonFX motor;

  /** Creates a new Shooter. */
  public Shooter() {
    motor = new TalonFX(SHOOTER_MOTOR);
    configureMotor();
  }

  public boolean isActivate() {
    return this.isActivated;
  }

  public double calculateOptimalTrajectory(double distance) {
    return Projectile.calculateRange(CURRENT_HOOD_ANGLE, distance);
  }

  public double getMotorRpms() {
    double velocity =
      (
        (
          (
            (this.motor.getSelectedSensorVelocity()) /
            Base.FULL_TALON_ROTATION_TICKS
          ) /
          Shooter.GEAR_MULTIPLYER
        ) *
        600
      );
    return Math.abs(velocity);
  }

  public boolean readyToShoot(double max, double threshold) {
    double velocity = getMotorRpms();
    return Math.abs(velocity - max) <= threshold;
  }

  public double getMotorOutputPercent() {
    return motor.getMotorOutputPercent();
  }

  public void command(double power) {
    // apply power
    motor.set(ControlMode.PercentOutput, power);
    // when power is being applied:  isActivated needs to be true
    this.isActivated = (power != 0) ? true : false;
  }

  public void commandRpm(double rpm) {
    double velo = ((rpm * 2048) * GEAR_MULTIPLYER) / 600;
    motor.set(ControlMode.Velocity, velo);
    // when power is being applied:  isActivated needs to be true
    this.isActivated = (rpm != 0) ? true : false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void configureMotor() {
    this.motor.setInverted(true);

    this.motor.setNeutralMode(NeutralMode.Coast);
    this.motor.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor,
        ShooterConstants.kPIDLoopIdx,
        ShooterConstants.kTimeoutMs
      );
    //this.motor.setSensorPhase(true);  //NOT NEEDED SINCE ITS INTEGRATED SENSOR
    this.motor.configClosedloopRamp(RAMP_RATE);
    this.motor.configNominalOutputForward(
        NOMINAL_FORWARD,
        ShooterConstants.kTimeoutMs
      );
    this.motor.configNominalOutputReverse(
        NOMINAL_REVERSE,
        ShooterConstants.kTimeoutMs
      );
    this.motor.configPeakOutputForward(
        PEAK_FORWARD,
        ShooterConstants.kTimeoutMs
      );
    this.motor.configPeakOutputReverse(
        PEAK_REVERSE,
        ShooterConstants.kTimeoutMs
      );
    this.motor.configAllowableClosedloopError(
        ShooterConstants.kPIDLoopIdx,
        ShooterConstants.ALLOWABLE_CLOSELOOP_ERROR,
        ShooterConstants.kTimeoutMs
      );
    this.motor.config_kF(
        ShooterConstants.kPIDLoopIdx,
        ShooterConstants.kGains.kF,
        ShooterConstants.kTimeoutMs
      );
    this.motor.config_kP(
        ShooterConstants.kPIDLoopIdx,
        ShooterConstants.kGains.kP,
        ShooterConstants.kTimeoutMs
      );
    this.motor.config_kI(
        ShooterConstants.kPIDLoopIdx,
        ShooterConstants.kGains.kI,
        ShooterConstants.kTimeoutMs
      );
    this.motor.config_kD(
        ShooterConstants.kPIDLoopIdx,
        ShooterConstants.kGains.kD,
        ShooterConstants.kTimeoutMs
      );
  }

  public void configFeedForward(double ff) {
    this.motor.config_kF(
        ShooterConstants.kPIDLoopIdx,
        ff,
        ShooterConstants.kTimeoutMs
      );
  }

  public void configKp(double Kp) {
    this.motor.config_kP(
        ShooterConstants.kPIDLoopIdx,
        Kp,
        ShooterConstants.kTimeoutMs
      );
  }
}
