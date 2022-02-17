// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  TalonFX motor = new TalonFX(36);

  /** Creates a new Shooter. */
  public Shooter() {
    motor.setNeutralMode(NeutralMode.Coast);
  }

  public void command(double power) {
    motor.set(ControlMode.PercentOutput, power);

    SmartDashboard.putNumber(
      "velocity",
      ((motor.getSelectedSensorVelocity() / 2048) * 10) * 60
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
