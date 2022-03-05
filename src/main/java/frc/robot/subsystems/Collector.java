// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase {

  public static int COLLECTOR_MOTOR_PORT = 10;
  public static double COLLECTOR_MOTOR_FULL_SPEED = 1;

  // TalonFX collectorMotor;
  Compressor compressor;

  /** Creates a new Collector. */
  public Collector(Compressor compressor) {
    // collectorMotor = new TalonFX(Collector.COLLECTOR_MOTOR_PORT);
    this.compressor = compressor;
  }

  public void SetSpeed(double speed) {
    // collectorMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
