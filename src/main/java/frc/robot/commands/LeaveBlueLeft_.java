// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Base;

@Deprecated
public class LeaveBlueLeft_ extends CommandBase {

  Base base;

  enum states {
    INIT,
    LEAVE_AND_GRAB_BALL,
    FACE_GOAL,
    SHOOT,
    STOP,
  }

  states currentState = states.INIT;

  private double LEAVE_AND_GRAB_BALL_POSITION = 50;
  private double LEAVE_AND_GRAB_BALL_ANGLE = 0;

  /** Creates a new AutoGrabBallShoot. */
  public LeaveBlueLeft_(Base base) {
    // Use addRequirements() here to declare subsystem dependencies

    this.base = base;
    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (currentState) {
      case INIT:
        /**
         * the reason why the gyro and drive encoders are not reset is
         * because initialize does not always run before execute starts,
         * causing the robot to skip steps because it was moved after initializion ran.
         */
        base.zeroGyro();
        base.resetDriveMotors();
        currentState = states.LEAVE_AND_GRAB_BALL;
        break;
      case LEAVE_AND_GRAB_BALL:
        if (
          base.driveStraight(
            LEAVE_AND_GRAB_BALL_POSITION,
            LEAVE_AND_GRAB_BALL_ANGLE
          )
        ) {
          currentState = states.FACE_GOAL;
        }
        break;
      case FACE_GOAL:
        if (base.rotate(180)) {
          currentState = states.STOP;
        }
        break;
      case SHOOT:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentState == states.STOP;
  }
}
