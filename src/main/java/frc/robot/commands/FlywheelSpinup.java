// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class FlywheelSpinup extends CommandBase {

  /** Creates a new FlywheelSpinup. */

  int BUTTON_ID;

  double REV_UP_SPEED = 1;
  double DEADBAND = 0.05;

  Joystick joystick;
  Shooter m_shooter;

  public FlywheelSpinup(Shooter m_shooter, Joystick joystick, int BUTTON_ID) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.BUTTON_ID = BUTTON_ID;
    this.m_shooter = m_shooter;
    this.joystick = joystick;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.command(this.REV_UP_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return joystick.getRawButtonReleased(BUTTON_ID);
  }
}
