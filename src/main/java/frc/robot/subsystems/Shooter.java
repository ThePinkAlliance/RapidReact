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

  TalonFX motor = new TalonFX(36);

  /** Creates a new Shooter. */
  public Shooter() {
    motor.setNeutralMode(NeutralMode.Coast);
    motor.config_kP(0, 1);
    motor.config_kI(0, 0);
    motor.config_kD(0, 0);
    motor.configVelocityMeasurementPeriod(
      SensorVelocityMeasPeriod.Period_2Ms,
      100
    );
  }

  public void revUp(double velocity) {
    motor.set(ControlMode.Velocity, velocity);
  }

  public void command(double power) {
    motor.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
