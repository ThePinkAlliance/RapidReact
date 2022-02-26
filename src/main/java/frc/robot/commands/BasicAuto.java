// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Base;

public class BasicAuto extends CommandBase {

  private boolean forwardFinished = false;
  private boolean rotateFinished = false;
  private Base m_base;

  /** Creates a new BasicAuto. */
  public BasicAuto(Base m_base) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_base = m_base;

    addRequirements(m_base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_base.resetDriveMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!forwardFinished) {
      forwardFinished = m_base.driveStraight(36, 0);
    }
    // if (forwardFinished) {
    //   rotateFinished = m_base.rotate(75);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return forwardFinished;
  }
}
