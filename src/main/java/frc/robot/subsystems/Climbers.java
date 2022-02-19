// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ClimberModule;

public class Climbers extends SubsystemBase {

  // The port range for the climbers is 40-49
  public ClimberModule leftModuleOne = new ClimberModule(40, 41);
  public ClimberModule leftModuleTwo = new ClimberModule(42, 43);
  public ClimberModule rightModuleOne = new ClimberModule(44, 45);
  public ClimberModule rightModuleTwo = new ClimberModule(46, 47);

  /** Creates a new Climbers. */
  public Climbers() {}

  public void commandAll(
    double leftOne,
    double leftTwo,
    double rightOne,
    double rightTwo
  ) {
    leftModuleOne.setPosition(leftOne);
    leftModuleTwo.setPosition(leftTwo);
    rightModuleOne.setPosition(rightOne);
    rightModuleTwo.setPosition(rightTwo);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
