// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ClimberModule;

public class Climbers extends SubsystemBase {

  public ClimberModule leftModuleOne = new ClimberModule(0, 0);
  public ClimberModule leftModuleTwo = new ClimberModule(0, 0);
  public ClimberModule rightModuleOne = new ClimberModule(0, 0);
  public ClimberModule rightModuleTwo = new ClimberModule(0, 0);

  /** Creates a new Climbers. */
  public Climbers() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
