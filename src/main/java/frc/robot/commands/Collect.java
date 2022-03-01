// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;

public class Collect extends CommandBase {

  Collector m_collector;

  /** Creates a new Collect. */
  public Collect(Collector collecter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(collecter);

    this.m_collector = collecter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //set motor to speed set in constants so we can change
    this.m_collector.SetSpeed(Collector.COLLECTOR_MOTOR_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stop motor so collector no longer runs
    this.m_collector.SetSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
