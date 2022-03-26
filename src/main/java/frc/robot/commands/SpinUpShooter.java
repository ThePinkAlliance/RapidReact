// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Shooter;

public class SpinUpShooter extends CommandBase {

  private Shooter m_shooter;
  private Joystick joystick;
  private double rpm;
  private int buttonId;
  

  public SpinUpShooter(
    Shooter m_shooter,
    double rpm,
    int buttonId,
    Joystick joystick
  ) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_shooter = m_shooter;
    this.rpm = rpm;
    this.buttonId = buttonId;
    this.joystick = joystick;

    addRequirements(this.m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rpm = SmartDashboard.getNumber(Dashboard.DASH_SHOOTER_TARGET_RPMS, rpm);
    boolean ready = m_shooter.readyToShoot(rpm, 100);
    SmartDashboard.putBoolean(Dashboard.DASH_SHOOTER_READY, ready);
    this.m_shooter.commandRpm(rpm);
    SmartDashboard.putNumber(Dashboard.DASH_SHOOTER_VELOCITY, this.m_shooter.getMotorOutputPercent());
    SmartDashboard.putNumber(Dashboard.DASH_SHOOTER_RPMS, this.m_shooter.getMotorRpms());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_shooter.command(0);
    SmartDashboard.putBoolean(Dashboard.DASH_SHOOTER_READY, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (joystick.getRawButton(buttonId) != true);
  }
}
