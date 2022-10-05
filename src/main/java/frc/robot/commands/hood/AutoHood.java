// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hood;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Debug;
import frc.robot.HoodConstants;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Hood;

public class AutoHood extends CommandBase {

  Hood m_hood;
  Joystick joystick;
  double position;
  int buttonId;

  /** Creates a new CommandHood. */
  public AutoHood(Hood m_hood, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_hood = m_hood;
    this.position = position;

    addRequirements(m_hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double p = HoodConstants.kGains.kP;
    double i = HoodConstants.kGains.kI;
    double d = HoodConstants.kGains.kD;

    this.m_hood.setPID(p, i, d);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ticks = position;
    // Enables the speed controller closed loop control.
    // This command does not have to continue after setting this on.
    this.m_hood.setPosition(ticks);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Debug.putNumber(
        Dashboard.DASH_HOOD_OUTPUT,
        this.m_hood.getHoodPower());
    Debug.putNumber(
        Dashboard.DASH_HOOD_DRAW,
        this.m_hood.getCurrentDraw());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This command sets the closed loop control of the hood to ON. Does not need to
    // continue.
    return true;
  }
}
