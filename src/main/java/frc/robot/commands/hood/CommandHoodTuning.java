// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hood;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.HoodConstants;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Hood;

public class CommandHoodTuning extends CommandBase {

  Hood m_hood;
  Joystick joystick;

  int buttonId;

  /** Creates a new CommandHood. */
  public CommandHoodTuning(
      Hood m_hood,
      Joystick joystick,
      int buttonId) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_hood = m_hood;
    this.buttonId = buttonId;
    this.joystick = joystick;

    addRequirements(m_hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Get dashboard value if overriden otherwise use defaults.
    double p = SmartDashboard.getNumber(
        Dashboard.DASH_HOOD_P,
        HoodConstants.kGains.kP);
    double i = SmartDashboard.getNumber(
        Dashboard.DASH_HOOD_I,
        HoodConstants.kGains.kI);
    double d = SmartDashboard.getNumber(
        Dashboard.DASH_HOOD_D,
        HoodConstants.kGains.kD);

    System.out.println("p: " + p + ", i: " + i + ", d: " + d);

    this.m_hood.setPID(p, i, d);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double ticks = SmartDashboard.getNumber(
        Dashboard.DASH_HOOD_TICKS,
        HoodConstants.IDLE_TICK_COUNT);

    this.m_hood.setPosition(ticks);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_hood.disableCloseLoopControl();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !this.joystick.getRawButton(buttonId);
  }
}
