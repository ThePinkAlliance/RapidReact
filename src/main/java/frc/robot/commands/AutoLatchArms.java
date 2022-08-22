// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climbers;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;


public class AutoLatchArms extends CommandBase {
  private Climbers m_climbers;
  private Timer m_timer;
  private double MAX_SECONDS = 1.5;

  /** Creates a new AutoLatchArms. */
  public AutoLatchArms(Climbers m_climbers) {
    // Use addRequirements() here to declare subsystem dependencies.
  
    this.m_climbers = m_climbers;
    this.m_timer = new Timer();

    this.m_timer.reset();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climbers.closeLongArms();
    m_climbers.closeShortArms();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("auto latch arms terminated: " + interrupted);

    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() >= MAX_SECONDS;
  }
}
