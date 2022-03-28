package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;

public class AutoCollectGroup extends CommandBase {

  Collector m_collector;
  boolean bIntake;
  double MAX_SECONDS;
  Timer timer;

  /**
   * NOTE bIntake won't be updated if this command is not being called repeatedly
   * @param m_collector
   * @param joystick
   * @param JOYSTICK_BUTTON
   * @param bIntake
   */
  public AutoCollectGroup(
    Collector m_collector,
    double MAX_SECONDS,
    boolean bIntake
  ) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_collector = m_collector;
    this.MAX_SECONDS = MAX_SECONDS;
    this.bIntake = bIntake;

    this.timer = new Timer();
    this.timer.reset();

    addRequirements(m_collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.timer.reset();
    this.timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (bIntake == false) {
      this.m_collector.SetSpeedCollector(-Collector.COLLECTOR_MOTOR_FULL_SPEED);
      this.m_collector.SetSpeedTowerForOverride(
          -Collector.TOWER_MOTOR_FULL_SPEED
        );
    } else {
      this.m_collector.SetSpeedCollector(Collector.COLLECTOR_MOTOR_FULL_SPEED);
      this.m_collector.SetSpeedTowerForOverride(
          Collector.TOWER_MOTOR_FULL_SPEED
        );
    }
    this.m_collector.setSolenoid(Value.kForward);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_collector.SetSpeedCollector(0);
    this.m_collector.SetSpeedTowerForOverride(0);
    this.m_collector.setSolenoid(Value.kReverse);
    this.timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(this.MAX_SECONDS);
  }
}
