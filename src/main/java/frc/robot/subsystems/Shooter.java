// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  public static final double SHOOTER_POWER_CLOSE_HIGH = 0.65;
  public static final double SHOOTER_POWER_CLOSE_LOW = 0.75;
  public static final double SHOOTER_POWER_CLOSE_DEFAULT =
    SHOOTER_POWER_CLOSE_HIGH;

  private final int SHOOTER_MOTOR = 30;

  private boolean isActivated = false;

  TalonFX motor = new TalonFX(SHOOTER_MOTOR);

  /** Creates a new Shooter. */
  public Shooter() {
    motor.setNeutralMode(NeutralMode.Coast);
  }

  public void revUp(double velocity) {
    motor.set(ControlMode.Velocity, velocity);
  }

  public boolean isActivate() {
    return this.isActivated;
  }

  public boolean readyToShoot() {
    double velocity =
      (
        (
          (this.motor.getSelectedSensorVelocity()) /
          Base.FULL_TALON_ROTATION_TICKS
        ) *
        600
      );
    double max = 5859;
    double threshold = 100;

    return Math.abs(velocity - max) >= threshold;
  }

  public boolean readyToShoot(double threshold) {
    double velocity =
      (
        (
          (this.motor.getSelectedSensorVelocity()) /
          Base.FULL_TALON_ROTATION_TICKS
        ) *
        600
      );
    double max = 5859;

    return Math.abs(velocity - max) >= threshold;
  }

  public boolean readyToShoot(double threshold, double max) {
    double velocity =
      (
        (
          (this.motor.getSelectedSensorVelocity()) /
          Base.FULL_TALON_ROTATION_TICKS
        ) *
        600
      );

    return Math.abs(velocity - max) >= threshold;
  }

  public void toggle(double power) {
    if (this.isActivated) {
      motor.set(ControlMode.PercentOutput, 0);
      this.isActivated = false;
    } else {
      motor.set(ControlMode.PercentOutput, power);
      this.isActivated = true;
    }
  }

  public void toggle() {
    this.toggle(Shooter.SHOOTER_POWER_CLOSE_DEFAULT);
  }

  public void command(double power) {
    // apply power
    motor.set(ControlMode.PercentOutput, power);
    // when power is being applied:  isActivated needs to be true
    this.isActivated = (power != 0) ? true : false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
