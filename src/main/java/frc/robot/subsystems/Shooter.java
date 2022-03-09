// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShooterConstants;

public class Shooter extends SubsystemBase {

  public static final double SHOOTER_POWER_CLOSE_HIGH = 0.75;
  public static final double SHOOTER_POWER_CLOSE_LOW = 0.75;
  public static final double SHOOTER_POWER_CLOSE_DEFAULT =
    SHOOTER_POWER_CLOSE_HIGH;
  
  private double RAMP_RATE = 0;
  private double NOMINAL_FORWARD = 0;
  private double NOMINAL_REVERSE = 0;
  private double PEAK_FORWARD = 1;
  private double PEAK_REVERSE = -1;
  private final int SHOOTER_MOTOR = 30;
  private boolean isActivated = false;
  private TalonFX motor;

  /** Creates a new Shooter. */
  public Shooter() {
    motor = new TalonFX(SHOOTER_MOTOR);
    configureMotor();
  }

  public boolean isActivate() {
    return this.isActivated;
  }

  public double getMotorRpms() {
    double velocity =
      (
        (
          (this.motor.getSelectedSensorVelocity()) /
          Base.FULL_TALON_ROTATION_TICKS
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
    double velo = (rpm * 248) / 600;
    motor.set(ControlMode.Velocity, velo); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

private void configureMotor() {
    this. motor.setNeutralMode(NeutralMode.Coast);
    this.motor.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor,
        ShooterConstants.kPIDLoopIdx,
        ShooterConstants.kTimeoutMs
      );
    //this.motor.setSensorPhase(true);  //NOT NEEDED SINCE ITS INTEGRATED SENSOR

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

}
