// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climbers;

public class AutoClimb extends CommandBase {

  Climbers m_climbers;

  enum states {
    STEP_1,
    STEP_2,
    STEP_3,
    STEP_4,
  }

  states currentClimbState = states.STEP_1;

  /** Creates a new AutoClimb. */
  public AutoClimb(Climbers m_climbers) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_climbers = m_climbers;

    addRequirements(m_climbers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (currentClimbState) {
      case STEP_1:
        m_climbers.leftOutModule.setPosition(7028);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
