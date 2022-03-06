// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Tower;

public class CollectTowerGroup extends CommandBase {

  Collector m_collector;
  Joystick joystick;
  int JOYSTICK_BUTTON;
  boolean bIntake;

  /**
   * NOTE bIntake won't be updated if this command is not being called repeatedly
   * @param m_collector
   * @param joystick
   * @param JOYSTICK_BUTTON
   * @param bIntake
   */
  public CollectTowerGroup(
    Collector m_collector,
    Joystick joystick,
    int JOYSTICK_BUTTON,
    boolean bIntake
  ) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_collector = m_collector;
    this.joystick = joystick;

    this.JOYSTICK_BUTTON = JOYSTICK_BUTTON;
    this.bIntake = bIntake;

    addRequirements(m_collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (bIntake == false) {
      this.m_collector.SetSpeed(-Collector.COLLECTOR_MOTOR_FULL_SPEED);
    } else this.m_collector.SetSpeed(Collector.COLLECTOR_MOTOR_FULL_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_collector.SetSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return joystick.getRawButtonReleased(JOYSTICK_BUTTON);
  }
}
