// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Unit;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TempTower extends SubsystemBase {

  private Rev2mDistanceSensor sensor;

  /** Creates a new TempBase. */
  public TempTower() {
    this.sensor = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kOnboard);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double dist = this.sensor.getRange(Unit.kInches);
  }
}
