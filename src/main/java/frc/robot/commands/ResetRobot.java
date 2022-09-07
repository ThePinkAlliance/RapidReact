// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

public class ResetRobot extends CommandBase {
  Hood hood;
  Timer timer;

  /** Creates a new ResetRobot. */
  public ResetRobot(Hood hood) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.hood = hood;

    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hood.commandHood(0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (hood.getCurrentDraw() >= 19.0) {
      timer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.commandHood(0);
    hood.resetHoodEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(0.3);
  }
}
