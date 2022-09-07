// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hood;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

public class DisableHoodControl extends CommandBase {

  Hood m_hood;
  Joystick joystick;

  /** Creates a new CommandHood. */
  public DisableHoodControl(Hood m_hood) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_hood = m_hood;
    addRequirements(m_hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Enables the speed controller closed loop control.
    // This command does not have to continue after setting this on.
    this.m_hood.disableCloseLoopControl();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This command set simply disables the closed loop control of the hood and
    // leaves it where it is.
    return true;
  }
}
