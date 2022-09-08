// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climbers;

public class CommandClimbers extends CommandBase {

  double positionShort;
  double positionLong;
  Climbers m_climbers;

  /** Creates a new CommandClimbers. */
  public CommandClimbers(
      Climbers m_climbers,
      double positionShort,
      double positionLong) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_climbers = m_climbers;

    this.positionLong = positionLong;
    this.positionShort = positionShort;

    addRequirements(m_climbers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climbers.longClimberModule.setPosition(positionLong);
    m_climbers.shortClimberModule.setPosition(positionShort);

    // SmartDashboard.putNumber(
    // "shortClimbers",
    // m_climbers.shortClimberModule.getPosition());

    // SmartDashboard.putNumber(
    // "longClimbers",
    // m_climbers.longClimberModule.getPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentLongPos = m_climbers.longClimberModule.getPosition();
    double currentShortPos = m_climbers.shortClimberModule.getPosition();
    double error = 100;
    boolean withInError = Math.abs(currentLongPos - positionLong) >= error &&
        Math.abs(currentShortPos - positionShort) >= error;

    return withInError;
  }
}
