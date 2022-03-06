// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  public static final double SHOOTER_POWER_DEFAULT = 0.75;
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
    this.toggle(Shooter.SHOOTER_POWER_DEFAULT);
  }

  public void command(double power) {
    motor.set(ControlMode.PercentOutput, power);

    if (power > 0) {
      this.isActivated = true;
    } else if (power == 0) {
      this.isActivated = false;
    } else if (power < -0) {
      this.isActivated = true;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
