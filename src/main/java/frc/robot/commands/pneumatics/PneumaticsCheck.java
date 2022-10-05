// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BooleanEntry;

public class PneumaticsCheck extends CommandBase {
  Compressor compressor;
  BooleanEntry pneumaticsReady;

  final double requiredPSI = 120;

  /** Creates a new PneumaticsCheck. */
  public PneumaticsCheck(Compressor compressor, BooleanEntry pneumaticsReady) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.pneumaticsReady = pneumaticsReady;
    this.compressor = compressor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double psi = this.compressor.getPressure();

    if (psi > requiredPSI) {
      this.pneumaticsReady.set(true);
    } else {
      this.pneumaticsReady.set(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    compressor.enableDigital();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.pneumaticsReady.get();
  }
}
