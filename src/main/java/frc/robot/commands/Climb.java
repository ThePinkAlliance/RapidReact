// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climbers;

public class Climb extends CommandBase {

  Climbers m_climbers;
  double targetPosition;

  enum SIDE {
    IN,
    OUT,
  }

  SIDE side;

  /** Creates a new AutoClimb. */
  public Climb(Climbers m_climbers, double position, SIDE side) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_climbers = m_climbers;
    this.targetPosition = position;
    this.side = side;

    addRequirements(m_climbers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (side == SIDE.IN) {
      this.m_climbers.leftInModule.setPosition(targetPosition);
      this.m_climbers.rightInModule.setPosition(targetPosition);
    } else if (side == SIDE.OUT) {
      this.m_climbers.leftOutModule.setPosition(targetPosition);
      this.m_climbers.rightOutModule.setPosition(targetPosition);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (side == SIDE.IN) {
      return (
        Math.abs(
          (
            (
              m_climbers.leftInModule.currentPosition() +
              m_climbers.rightInModule.currentPosition()
            ) /
            2.0
          ) -
          targetPosition
        ) <
        5.0
      );
    } else if (side == SIDE.OUT) {
      return (
        Math.abs(
          (
            (
              m_climbers.leftOutModule.currentPosition() +
              m_climbers.rightOutModule.currentPosition()
            ) /
            2.0
          ) -
          targetPosition
        ) <
        5.0
      );
    }

    return false;
  }
}
