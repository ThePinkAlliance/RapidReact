// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Unit;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TempTower extends SubsystemBase {

  private final I2C.Port port = I2C.Port.kOnboard;

  private final ColorSensorV3 colorSensor = new ColorSensorV3(port);

  private ShuffleboardTab debug = Shuffleboard.getTab("debug");

  /** Creates a new TempBase. */
  public TempTower() {}

  public ColorSensorV3 getColorSensor() {
    return this.colorSensor;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // double dist = this.sensor.getRange(Unit.kInches);

    if (colorSensor != null) {
      SmartDashboard.putNumber("blue", colorSensor.getRawColor().blue);
      SmartDashboard.putNumber("red", colorSensor.getRawColor().red);
      SmartDashboard.putNumber("green", colorSensor.getRawColor().green);
    }
    // debug.add("distance", dist);
  }
}
