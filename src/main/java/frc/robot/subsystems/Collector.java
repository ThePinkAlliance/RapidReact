// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase {

  public static int COLLECTOR_MOTOR_PORT = 10;
  public static double COLLECTOR_MOTOR_FULL_SPEED = 1;
  private TalonFX collectorMotor;
  private Solenoid solenoid;

  /** Creates a new Collector. */
  public Collector() {
    this.collectorMotor = new TalonFX(Collector.COLLECTOR_MOTOR_PORT);
    this.solenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);

    this.collectorMotor.setInverted(true);
  }

  public void dropCollector() {
    this.collectorMotor.set(ControlMode.PercentOutput, 1);
    this.solenoid.set(true);
  }

  public void setSolenoid(boolean on) {
    this.solenoid.set(on);
  }

  public void SetSpeed(double speed) {
    collectorMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
