// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ClimberModule;
import frc.robot.ClimberModule.SOLENOID_STATE;

public class Climbers extends SubsystemBase {

  // // The port range for the climbers is 40-59
  public ClimberModule longClimberModule = new ClimberModule(
    40,
    41,
    42,
    43,
    true,
    0
  );
  public ClimberModule shortClimberModule = new ClimberModule(
    44,
    45,
    46,
    47,
    true,
    0
  );

  /** Creates a new Climbers. */
  public Climbers() {}

  public void command(ClimberModule module, double power) {
    boolean contactedPole = module.contactedPole();

    if (!contactedPole) {
      module.setPower(power);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
