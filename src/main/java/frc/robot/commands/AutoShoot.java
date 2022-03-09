// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;

public class AutoShoot extends CommandBase {

  private double MAX_TIMER_SECS = 3;

  private Shooter m_shooter;
  private Collector m_collector;

  private double rpm;
  private boolean atRpm;

  private Timer timer;

  public AutoShoot(Shooter m_shooter, Collector m_collector, double rpm) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_collector = m_collector;
    this.m_shooter = m_shooter;
    this.rpm = rpm;

    this.timer = new Timer();
    this.atRpm = false;

    addRequirements(this.m_shooter, this.m_collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rpm = SmartDashboard.getNumber("shooter rpms", rpm);
    atRpm = m_shooter.readyToShoot(rpm, 100);

    if (atRpm) {
      this.m_collector.enableTowerOverride();
    } else {
      this.m_collector.disableTowerOverride();
    }

    this.m_collector.SetSpeedTower(Collector.TOWER_MOTOR_FULL_SPEED);
    this.m_shooter.commandRpm(rpm);
    SmartDashboard.putNumber(
      "shooter output percent:",
      this.m_shooter.getMotorOutputPercent()
    );
    SmartDashboard.putNumber("shooter rpms:", this.m_shooter.getMotorRpms());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_shooter.command(0);
    this.m_collector.disableTowerOverride();
    this.m_collector.SetSpeedTower(0);
    this.timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atRpm || timer.get() < MAX_TIMER_SECS;
  }
}
