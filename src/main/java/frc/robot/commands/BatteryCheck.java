// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BooleanEntry;

public class BatteryCheck extends CommandBase {
  BooleanEntry batterySufficient;
  PowerDistribution pdp;

  final double requiredVoltage = 11.9;

  /** Creates a new BatteryCheck. */
  public BatteryCheck(BooleanEntry batterySufficient) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.batterySufficient = batterySufficient;
    this.pdp = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double voltage = pdp.getVoltage();

    if (voltage >= requiredVoltage) {
      this.batterySufficient.set(true);
    } else {
      this.batterySufficient.set(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
