// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;

public class Shoot extends CommandBase {

  private Shooter m_shooter;
  private Joystick joystick;
  private Collector m_collector;
  private double power;
  private int buttonId;

  public Shoot(
    Shooter m_shooter,
    Collector m_collector,
    double power,
    int buttonId,
    Joystick joystick
  ) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_collector = m_collector;
    this.m_shooter = m_shooter;
    this.power = power;
    this.buttonId = buttonId;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean ready = m_shooter.readyToShoot();

    if (ready) {
      m_collector.enableTowerOverride();
    }

    m_collector.SetSpeedTower(Collector.TOWER_MOTOR_FULL_SPEED);
    m_shooter.command(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.command(0);

    m_collector.disableOverride();
    m_collector.SetSpeedTower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return joystick.getRawButtonReleased(this.buttonId);
  }
}
